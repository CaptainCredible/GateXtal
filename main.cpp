/*
  GateXtal — monophonic FM synth + sequencer (Mozzi)
  ESP32-S3 SuperMini port of the original ATmega32u4 version.
  By Daniel Lacey a.k.a. Captain Credible

  PINOUT (GPIO numbers as printed on the SuperMini):
    A button ..... 11   (all buttons to GND, internal pullups)
    B button ..... 12
    SHIFT button . 13   (held = alternate knob functions on every page)
    PAGE button .. 1   (single button, cycles the 6 pages:
                           MAIN / AMPENV / LFO / VERB / H4XX / SEQ)
    ARCADE button .. 43   (the "TX" pin — plain GPIO once booted; the boot ROM
                           chatters on it for ~100ms at reset, harmless for a
                           button. A 470R-1k series resistor is cheap insurance
                           against holding it during reset.)
    POTS ....... 2,3,4,5  (FM / haxx / attack / release — all ADC1)
    MIDI DIN in .... 44   (the "RX" pin, optional — RX only, TX stays free)
 
    PCM5102A:
      BCK ... 10
      LCK .... 8   (word select / LRCLK)
      DIN .... 9
      SCK .... GND (DAC generates its own master clock from BCK)

    SSD1306 OLED (I2C, 128x64, addr 0x3C):
      SDA .... 7
      SCL .... 6

  Changes from the 32u4 version:
    - Audio out via I2S to the PCM5102A instead of PWM (16-bit now!)
    - One PAGE button instead of +/- page buttons
    - SSD1306 OLED shows the current page + its four knob values. It is
      drawn by a FreeRTOS task pinned to core 0 (Arduino/Mozzi run on
      core 1), so the ~23ms I2C framebuffer push never blocks the DSP.
    - No LEDs, so the LED-pin external gate sync input is gone. Ext-clock
      mode = MIDI clock or stepping manually with the arcade button, as before.
    - USB-MIDI via the S3's native USB (enumerates as "GateXtal"); DIN MIDI in kept.
      Needs ARDUINO_USB_MODE=0 (USB-OTG/TinyUSB) — set in platformio.ini.
    - EEPROM is flash-emulated on ESP32: begin() + commit() required.π
*/

#include <Arduino.h>

// ------- Mozzi configuration: must come before any Mozzi include -------
#include <MozziConfigValues.h>
#define MOZZI_AUDIO_MODE   MOZZI_OUTPUT_I2S_DAC
#define MOZZI_AUDIO_BITS   16
#define MOZZI_AUDIO_CHANNELS MOZZI_STEREO // stereo out: dry stays centred, reverb is widened
#define MOZZI_I2S_PIN_BCK  10
#define MOZZI_I2S_PIN_WS   8
#define MOZZI_I2S_PIN_DATA 9
// (note: tried MOZZI_AUDIO_RATE 65536 to push Nyquist up for less FM aliasing,
// but Oscil's UPDATE_RATE template parameter is uint16_t, so 32768 is Mozzi's hard
// cap. 44100 is also out: MOZZI_CHECK_POW2 rejects non-power-of-2 rates.)
#define MOZZI_CONTROL_RATE 256 // powers of 2 please. 256 halves the MIDI-scan wait
                               // (~3.9ms) vs 128; seq tempo maths below are scaled to match

// Latency fix: Mozzi creates its I2S channel with IDF's stock
// I2S_CHANNEL_DEFAULT_CONFIG = 6 DMA buffers x 240 frames = 1440 samples,
// i.e. ~44ms of output latency at 32768Hz. Mozzi's implementation is
// header-compiled into this very translation unit (MozziGuts.h includes
// internal/MozziGuts.hpp), so redefining the macro here before <Mozzi.h>
// is enough. 4 x 64 = 256 samples ~= 8ms at 32768Hz. Field list must match
// the IDF original in driver/i2s_common.h exactly.
#include <driver/i2s_common.h>
#undef I2S_CHANNEL_DEFAULT_CONFIG
#define I2S_CHANNEL_DEFAULT_CONFIG(i2s_num, i2s_role) { \
    .id = i2s_num, \
    .role = i2s_role, \
    .dma_desc_num = 4, \
    .dma_frame_num = 64, \
    .auto_clear_after_cb = false, \
    .auto_clear_before_cb = false, \
    .intr_priority = 0, \
}

#include <Mozzi.h>
#include <Oscil.h>
#include <mozzi_midi.h>
#include <ADSR.h>
#include <ResonantFilter.h> // LowPassFilter lives here in Mozzi 2.x
#include <mozzi_rand.h>
#include <mozzi_fixmath.h>
#include <Line.h>

#include <EEPROM.h>
#include <MIDI.h>
#include <USB.h>
#include <USBMIDI.h>
#include "esp32-hal-tinyusb.h" // usb_persist_restart(): reboot into the ROM bootloader

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// TABLES
#include <tables/sin1024_int8.h>              // sine for oscillator
#include <tables/saw2048_int8.h>              // saw table for oscillator
#include <tables/triangle2048_int8.h>         // triangle table for oscillator
#include <tables/square_analogue512_int8.h>   // square table for oscillator
#include <tables/sin512_int8.h>               // lofi sine for LFO

#include "Freeverb.h" // needs MOZZI_AUDIO_RATE, so must come after Mozzi.h

// ------- PINS -------
#define PIN_ARCADE  43
#define PIN_MIDI_RX 44

#define PIN_OLED_SDA 7
#define PIN_OLED_SCL 6
#define OLED_ADDR    0x3C
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

int BUTTONS[4] = { 11, 12, 13, 1 }; // index -> GPIO
#define BTN_A     0 // pin 11
#define BTN_B     1 // pin 12
#define BTN_SHIFT 2 // pin 13 — held = alt knob functions
#define BTN_PAGE  3 // pin 1  — cycles pages

int KNOBS[4] = { 2, 3, 4, 5 };
#define FMknob      0
#define h4xxKnob    1
#define attackKnob  2
#define releaseKnob 3

// ------- EEPROM layout (flash-emulated) -------
#define EEPROM_SIZE   1536 // sequence + 10 preset slots
#define EE_SEQ_BASE   0    // steps 0..255
#define EE_ADDR_LEN   256
#define EE_ADDR_MAGIC 257
#define EE_MAGIC      123  // flag so we can tell if EEPROM contains a sequence or mumbo jumbo

#define EE_PRESET_ADDR   260  // preset slots live above the sequence
#define EE_PRESET_STRIDE 96   // bytes per slot (struct is ~72, rounded up for future fields)
#define NUM_PRESETS      10   // 260 + 10*96 = 1220, fits EEPROM_SIZE with room to spare
#define PRESET_MAGIC     0x45 // bumped when the struct layout changes: old presets read as empty

struct Preset { // everything that makes the sound; saved/loaded on the SET page
	byte magic;
	byte carWave, modWave;
	byte lpfCutoff, lpfRes;
	byte mod_ratio;
	byte lfoDest, lfoWaveSelect;
	byte bit7;
	byte env2FM, env2Filt, env2Ratio; // ENV2 routing amounts (FM / filter / ratio)
	byte polyFilt; // 0 = PARA (one filter on the sum), 1 = POLY (filter per voice)
	int32_t fmIntensity;
	int16_t attack, decay, sustain, release;         // ENV1 (amp)
	int16_t e2Attack, e2Decay, e2Sustain, e2Release; // ENV2 (mod)
	float lfoRate, modDepth;
	float rvSize, rvDamp, rvMix;
	float oscVol, limThresh;
	float rvSpread; // reverb stereo width; appended last so old presets stay readable
};
static_assert(sizeof(Preset) <= EE_PRESET_STRIDE, "Preset struct outgrew its EEPROM slot");

// ------- STATE -------
int transpose = 0;
int octTranspose = 0;

byte midiClockStepSize = 24;

bool midiClockRunning = false;
unsigned long lastMidiTickMs = 0; // for the clock timeout in handleSequencer()
bool writeMode = false;
byte noteToWrite = 0;
byte octOffset = 0;
byte sequence[256] = { 40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51 };
unsigned int seqIncrement = 0; // counter to keep track of when next step should come
byte seqCurrentStep = 0;
byte midiClockTicks = 0;
byte midiSeqNoteLength = 23;
unsigned int seqNoteLength = 120; // in control ticks (256Hz) — was 60 at CONTROL_RATE 128
byte midiClockDivider = 1;
bool internalClockSelect = false; // 0 = Arcade only or midi clock, 1 = internal clock
unsigned int seqTempo = 240; // in control ticks (256Hz) — was 120 at CONTROL_RATE 128
byte seqLength = 16;
byte writeOctSelect = 3;
byte noteSelect = 0;
byte prevNoteSelect = 0;
bool refreshWriteNotePing = false;

Line <Q16n16> aInterpolate;
int mozziRaw[4] = { 0,0,0,0 };
int oldMozziRaw[4] = { 0,0,0,0 };
bool buttStates[4] = { false, false, false, false };
bool oldButtStates[4] = { false, false, false, false };
bool ArcadeState = false;
bool oldArcadeState = false;
bool noteIsOn = false; // keep track of number of playing notes

byte pageState = 0;
// page order (the PAGE button cycles these). Reorder the whole UI by renumbering
// here — every switch/if below keys off these names, not raw case numbers.
enum Page { PG_MAIN = 0, PG_ENV, PG_LFO, PG_VERB, PG_HAXX, PG_SEQ, PG_COUNT };
int8_t lfoOutput = 0;  // value to store current offset from root
int mod_ratio = 3;
long fm_intensity = 0;
byte waveformselect = 0;
byte lastNote = 0;
float modDepth = 0;
float freeq = 0;
bool offsetOn = false;
byte lfoDest = 0;
byte lpfCutoff = 100;
byte lfoMode = 0;      // LFO shape (A cycles): 0 = sine..saw morph,
                       // 1 = RND (slewed random), 2 = STAT (silence + crackle)
int statVal = 128;     // STAT mode running level, 0..255 (128 = centre/silence)
unsigned long rndTimer = 0;
long int rndFreq = 0;
Q16n16 HDlfoOutputBuffer = 0; // this is a big type for slew manipulation
byte arcadeNote = 0;
bool knobLock[4] = { true, true, true, true };
int lockAnchor[4] = { -99,-99,-99,-99 };
int lockThresh = 50;
float knobSmooth[4] = { 0, 0, 0, 0 }; // adaptive-EMA state for the pot filter, in raw ADC units

// shadow copies for the OLED of settings that otherwise only live inside the
// Mozzi objects (initialised to the values set in setup())
int dispAttack = 100, dispDecay = 200, dispSustain = 240, dispRelease = 200;
float lfoRate = 1;
byte lpfRes = 200;
byte carWave = 0, modWave = 0;

MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);
USBMIDI usbMIDI;     // native USB-MIDI (global: registers the TinyUSB interface early)
USBCDC SerialCDC(0); // USB serial port for debugging + upload auto-reset
                     // (the core's USBSerial global only exists with CDC_ON_BOOT=1,
                     // which we avoid so USB.productName() can apply)

// ------- VOICES (4-voice polyphony, one shared LFO) -------
#define NUM_VOICES 4
struct Voice {
	Oscil <SIN1024_NUM_CELLS, MOZZI_AUDIO_RATE> car{ SIN1024_DATA }; // carrier
	Oscil <SIN1024_NUM_CELLS, MOZZI_AUDIO_RATE> mod{ SIN1024_DATA }; // FM modulator
	ADSR <MOZZI_CONTROL_RATE, MOZZI_AUDIO_RATE> env;                 // amp envelope (ENV1)
	ADSR <MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> env2;              // mod envelope (ENV2), per note
	LowPassFilter16 lpf;    // own filter, used in POLY filter mode
	byte note = 0;          // MIDI note this voice is sounding (as received, pre-transpose)
	bool active = false;    // gate: noteOn seen, no noteOff yet (env may still be releasing)
	float freq = 0;         // base carrier freq incl. oct transpose
	unsigned long age = 0;  // allocation order, for voice stealing
	long fmNow = 0;         // fm_intensity + this voice's ENV2 FM contribution
};
Voice voices[NUM_VOICES];
unsigned long voiceAge = 0;

Oscil <SIN512_NUM_CELLS, MOZZI_CONTROL_RATE> LFO(SIN512_DATA); // one LFO shared by all voices
LowPassFilter16 lpf; // the shared filter, used in PARA filter mode (16-bit variant so it
                     // doesn't crush the wide path; lpfCutoff/lpfRes stay 0-255, applied <<8)
bool polyFilter = false; // VERB page knob 4: PARA = one filter on the sum (classic paraphonic
                         // squelch), POLY = each voice through its own filter (per-note plucks)
Freeverb reverb; // ~37KB of delay lines, lives in .bss
float rvSize = 0.72f, rvDamp = 0.45f; // VERB page values (defaults match Freeverb ctor)
float rvMix = 0.5f;   // VERB page knob 3: single wet/dry crossfade (dry full below center, wet full above)
float rvSpread = 1.0f; // VERB page knob 4: reverb stereo width (0 = mono, 1 = natural, up to 8 = absurd)
float dryNow = 1.0f;  // effective dry gain this tick, from the rvMix law
float wetBase = 1.0f; // base reverb input gain, from the rvMix law (ENV2 sends ride on top)

// ENV2: per-voice modulation envelope, triggered/released with each voice's own gate.
// Edited on the ENV page via B2 (all voices in lockstep, like ENV1); B3 opens the
// routing view where the knobs set the four destination amounts.
byte envSelect = 0;        // ENV page: 0 = knobs edit ENV1 (amp), 1 = knobs edit ENV2
bool envRouteView = false; // ENV page: B3 toggles the ENV2 routing view
byte env2FM = 0, env2Filt = 0, env2Ratio = 0;   // routing amounts, 0-255 (reverb send removed)
int e2Attack = 100, e2Decay = 200, e2Sustain = 240, e2Release = 200; // OLED shadows for ENV2
bool bit7Mode = false; // H4XX page: crush the synth back to the old AVR's ~7-bit resolution, for crunch

// output stage (post reverb mix): peak limiter into a tanh-shaped saturator.
// Single notes pass untouched, stacked voices get transparently ducked, and
// whatever is driven past that bends smoothly instead of clipping hard.
float limEnv = 0;              // peak envelope follower
float limThresh = 44000.0f;    // limit level (SET page knob 4), of the ±65536 output range
float oscVol = 1.0f;           // pre-filter voice-mix gain (SET page knob 3): >1 = drive
#define LIM_RELEASE 0.99969f   // per-sample release: falls to 37% in ~100ms at 32768Hz

byte presetMsg = 0;                // 1 = saved, 2 = loaded, 3 = no preset found
unsigned long presetMsgUntil = 0;  // OLED shows the message until this millis()
byte presetSelMode = 0;            // 0 = closed, 1 = choosing a save slot, 2 = choosing a load slot
byte presetSelSlot = 0;            // slot the selector is pointing at (0..NUM_PRESETS-1)
bool presetUsed[NUM_PRESETS] = { false }; // which slots hold a patch (scanned at boot)

// ------- forward declarations (this is a .cpp, no .ino magic prototypes) -------
void HandleNoteOn(byte note, byte velocity);
void HandleNoteOff(byte note, byte velocity);
void playNextStep();
void writeToSeq();
void setWriteNote(byte thisNote);
void seqCheckButts();
void handleSequencer();
void writeSeqToEeprom();
void readSeqFromEeprom();
void lockKnobs();

///////////////////////
// VOICE / MIDI HANDLING
///////////////////////

// Note/MIDI activity light on the SuperMini's onboard neopixel (GPIO48).
// rgbLedWrite() drives it via RMT, same as digitalWrite(LED_BUILTIN) in the
// Arduino IDE; RGB_BUILTIN comes from the esp32s3 board variant.

void writeNoteLED(byte note) { // color the LED by the pressed note's pitch class:
                               // red (C) ramping through orange to yellow (B),
                               // one full red->yellow sweep per octave
#ifdef RGB_BUILTIN
	byte g = (uint16_t)(note % 12) * RGB_BRIGHTNESS / 11; // green rises 0..full
	rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, g, 0);
#endif
}

void writeLED(bool state) { // used for note-off (green kept for any non-note use)
#ifdef RGB_BUILTIN
	rgbLedWrite(RGB_BUILTIN, 0, state ? RGB_BRIGHTNESS : 0, 0);
#endif
}

Voice* allocVoice(byte note) {
	// same note still sounding: retrigger that voice instead of doubling it
	for (int i = 0; i < NUM_VOICES; i++) {
		if (voices[i].active && voices[i].note == note) return &voices[i];
	}
	// otherwise a completely idle voice (envelope finished)
	for (int i = 0; i < NUM_VOICES; i++) {
		if (!voices[i].active && !voices[i].env.playing()) return &voices[i];
	}
	// otherwise the oldest released-but-still-ringing tail
	Voice* best = NULL;
	for (int i = 0; i < NUM_VOICES; i++) {
		if (!voices[i].active && (!best || voices[i].age < best->age)) best = &voices[i];
	}
	if (best) return best;
	// all four gates held: steal the oldest
	best = &voices[0];
	for (int i = 1; i < NUM_VOICES; i++) {
		if (voices[i].age < best->age) best = &voices[i];
	}
	return best;
}

void HandleNoteOn(byte note, byte velocity) {
	if (note != 0) { // if not zero
		int octedNote = constrain(note + (octTranspose * 12), 0, 127);
		Voice* v = allocVoice(note);
		v->note = note;
		v->active = true;
		v->age = ++voiceAge;
		v->freq = mtof(float(octedNote));
		v->car.setFreq(v->freq);
		v->env.noteOn();
		v->env2.noteOn(); // this voice's own mod envelope
		noteIsOn = true;
		writeNoteLED(note);
		lastNote = note;
	}
}

// re-pitch every held voice from its own note (replaces the old mono legato():
// slides to the new octave without retriggering the envelopes; idle voices
// pick the new transpose up at their next noteOn)
void applyOctTranspose() {
	for (int i = 0; i < NUM_VOICES; i++) {
		if (voices[i].active) {
			int octedNote = constrain(voices[i].note + (octTranspose * 12), 0, 127);
			voices[i].freq = mtof(float(octedNote));
			voices[i].car.setFreq(voices[i].freq);
		}
	}
}

void HandleNoteOff(byte note, byte velocity) {
	bool anyHeld = false;
	for (int i = 0; i < NUM_VOICES; i++) {
		if (voices[i].active && voices[i].note == note) {
			voices[i].active = false;
			voices[i].env.noteOff();
			voices[i].env2.noteOff(); // both envelopes follow this voice's gate
		}
		anyHeld |= voices[i].active;
	}
	noteIsOn = anyHeld;
	if (!anyHeld) writeLED(false);
}

// --- SysEx protocol -------------------------------------------------------
// All GateXtal SysEx shares the header  F0 7D 47 58 <cmd> ...  F7
//   7D       = the educational / non-commercial manufacturer ID
//   47 58    = "GX"
//   <cmd>    = one ASCII byte selecting the operation:
//     'B' 0x42  reboot to bootloader   F0 7D 47 58 42 F7            (host->dev)
//     'D' 0x44  dump request           F0 7D 47 58 44 <slot> F7     (host->dev)
//                 slot 0..9 dumps that slot; 0x7F dumps every used slot.
//     'P' 0x50  preset payload         F0 7D 47 58 50 <slot> <nibbles..> F7
//                 sent by the device in reply to a dump, or by the host to
//                 store a preset into a slot. The Preset struct is carried
//                 low-nibble-first: each raw byte becomes two 4-bit data
//                 bytes so everything stays inside SysEx's 7-bit limit.
const byte GX_ID0 = 0x7D, GX_ID1 = 0x47, GX_ID2 = 0x58;
#define GX_CMD_BOOT    0x42
#define GX_CMD_DUMPREQ 0x44
#define GX_CMD_PRESET  0x50
#define GX_DUMP_ALL    0x7F

// backwards-compatible name for the old 6-byte bootloader magic
const byte BOOT_SYSEX[6] = { 0xF0, GX_ID0, GX_ID1, GX_ID2, GX_CMD_BOOT, 0xF7 };

void rebootToBootloader() {
	usb_persist_restart(RESTART_BOOTLOADER); // keeps USB alive across the reboot
}

// Send a raw SysEx byte stream (F0..F7) out over USB-MIDI, packed into the
// standard 4-byte USB-MIDI packets (CIN 4 = mid-stream, 5/6/7 = final 1/2/3
// bytes). Spins briefly if the TinyUSB FIFO is full rather than dropping data.
void sendSysExOut(const byte* d, int len) {
	int i = 0;
	while (i < len) {
		int rem = len - i;
		midiEventPacket_t p = { 0, 0, 0, 0 };
		if (rem > 3) { p.header = 0x04; p.byte1 = d[i]; p.byte2 = d[i + 1]; p.byte3 = d[i + 2]; i += 3; }
		else if (rem == 3) { p.header = 0x07; p.byte1 = d[i]; p.byte2 = d[i + 1]; p.byte3 = d[i + 2]; i += 3; }
		else if (rem == 2) { p.header = 0x06; p.byte1 = d[i]; p.byte2 = d[i + 1]; i += 2; }
		else { p.header = 0x05; p.byte1 = d[i]; i += 1; }
		for (int tries = 0; !usbMIDI.writePacket(&p) && tries < 20000; tries++) delayMicroseconds(50);
	}
}

// Read slot `slot` from EEPROM and, if it holds a patch, dump it as a 'P'
// message. Empty slots are silently skipped.
void dumpPreset(byte slot) {
	if (slot >= NUM_PRESETS) return;
	Preset p;
	EEPROM.get(EE_PRESET_ADDR + slot * EE_PRESET_STRIDE, p);
	if (p.magic != PRESET_MAGIC) return; // nothing saved in this slot
	const byte* raw = (const byte*)&p;
	byte out[7 + 2 * sizeof(Preset)];
	int n = 0;
	out[n++] = 0xF0; out[n++] = GX_ID0; out[n++] = GX_ID1; out[n++] = GX_ID2;
	out[n++] = GX_CMD_PRESET; out[n++] = slot;
	for (size_t k = 0; k < sizeof(Preset); k++) {
		out[n++] = raw[k] & 0x0F;
		out[n++] = (raw[k] >> 4) & 0x0F;
	}
	out[n++] = 0xF7;
	sendSysExOut(out, n);
}

// Decode a 'P' message and store the carried preset into its slot. Rejects
// wrong sizes / bad magic so a truncated or foreign SysEx can't corrupt a slot.
void storePresetFromSysEx(const byte* buf, int len) {
	byte slot = buf[5];
	if (slot >= NUM_PRESETS) return;
	int nib = len - 7; // strip F0 + 3 ID + cmd + slot + F7
	if (nib != (int)(2 * sizeof(Preset))) return;
	Preset p;
	byte* raw = (byte*)&p;
	for (size_t k = 0; k < sizeof(Preset); k++) {
		raw[k] = (buf[6 + 2 * k] & 0x0F) | ((buf[6 + 2 * k + 1] & 0x0F) << 4);
	}
	if (p.magic != PRESET_MAGIC) return; // not one of ours / stale layout
	EEPROM.put(EE_PRESET_ADDR + slot * EE_PRESET_STRIDE, p);
	EEPROM.commit();
	presetUsed[slot] = true;
	presetMsg = 4; // "SYSEX RX" footer hint
	presetMsgUntil = millis() + 1200;
}

// Dispatch a complete SysEx message (framing included) to the right handler.
void handleSysExMessage(const byte* buf, int len) {
	if (len == sizeof(BOOT_SYSEX) && memcmp(buf, BOOT_SYSEX, len) == 0) {
		rebootToBootloader();
		return;
	}
	if (len < 6) return;
	if (buf[0] != 0xF0 || buf[1] != GX_ID0 || buf[2] != GX_ID1 || buf[3] != GX_ID2) return;
	switch (buf[4]) {
		case GX_CMD_DUMPREQ:
			if (buf[5] == GX_DUMP_ALL) { for (byte s = 0; s < NUM_PRESETS; s++) dumpPreset(s); }
			else dumpPreset(buf[5]);
			break;
		case GX_CMD_PRESET:
			storePresetFromSysEx(buf, len);
			break;
	}
}

byte syxBuf[208]; // header(6) + slot + 2*sizeof(Preset) nibbles + F7, with headroom
byte syxLen = 0;

void syxByte(byte b) { // feed one byte of a USB-MIDI SysEx stream
	if (b == 0xF0) syxLen = 0; // message start
	if (syxLen < sizeof(syxBuf)) syxBuf[syxLen++] = b;
	if (b == 0xF7) { // message end: dispatch what we buffered
		handleSysExMessage(syxBuf, syxLen);
		syxLen = 0;
	}
}

void HandleDINSysEx(byte* data, unsigned length) { // includes the F0/F7 framing
	handleSysExMessage(data, (int)length);
}
// ---------------------------------------------------------------------------

void HandleDINNoteOn(byte channel, byte note, byte velocity) {
	HandleNoteOn(note, velocity);
	if (pageState == PG_SEQ && buttStates[BTN_B]) {
		noteToWrite = note;
		writeToSeq();
	}
}

void HandleDINNoteOff(byte channel, byte note, byte velocity) {
	HandleNoteOff(note, velocity);
}

void handleMidiClockTicks() {
	midiClockRunning = true;
	lastMidiTickMs = millis();
	midiClockTicks++;
	midiClockTicks = midiClockTicks % midiClockStepSize;
	if (midiClockTicks == 1) {
		playNextStep();
	}
	else if (midiClockTicks > midiSeqNoteLength && noteIsOn) { // passed note length and a note is on
		HandleNoteOff(sequence[seqCurrentStep], 0);
	}
}

void resetSeq() {
	seqCurrentStep = seqLength - 1;
	midiClockTicks = 0;
}

void handleMIDIClock() {
	if (!internalClockSelect) {
		handleMidiClockTicks();
	}
}

void handleMIDIClockStart() {
	if (!internalClockSelect) {
		resetSeq();
	}
}

void handleMIDIClockStop() {
	if (!internalClockSelect) {
		HandleNoteOff(sequence[seqCurrentStep], 0);
		midiClockRunning = false;
	}
}

void usbmidiprocessing() {
	midiEventPacket_t e;
	while (usbMIDI.readPacket(&e)) {
		byte cin = e.header & 0x0F; // code index number, same as the old MIDIUSB e.type

		// NOTE ON WITH VELOCITY GREATER THAN ZERO
		if (cin == 0x09 && e.byte3 > 0) {
			HandleNoteOn(e.byte2, e.byte3);
			if (pageState == PG_SEQ && buttStates[BTN_B]) {
				noteToWrite = e.byte2;
				writeToSeq();
			}
		}
		// USB NOTE OFF
		else if (cin == 0x08) {
			HandleNoteOff(e.byte2, e.byte3);
		}
		// NOTE ON W/ ZERO VELOCITY
		else if (cin == 0x09 && e.byte3 == 0) {
			if (!internalClockSelect) {
				HandleNoteOff(e.byte2, e.byte3);
			}
		}
		// sysex stream: CIN 4 = continue (3 bytes), 5/6/7 = end with 1/2/3 bytes
		else if (cin >= 0x04 && cin <= 0x07) {
			byte n = (cin == 0x05) ? 1 : (cin == 0x06) ? 2 : 3;
			syxByte(e.byte1);
			if (n >= 2) syxByte(e.byte2);
			if (n >= 3) syxByte(e.byte3);
		}
		// single-byte system realtime: clock / start / continue / stop
		else if (cin == 0x0F) {
			if (e.byte1 == 0xF8) { // clock tick
				if (!internalClockSelect) {
					handleMidiClockTicks();
				}
			}
			else if (e.byte1 == 0xFA || e.byte1 == 0xFB) { // start / continue
				if (!internalClockSelect) {
					resetSeq();
				}
			}
			else if (e.byte1 == 0xFC) { // stop (252, like the old e.m1 check)
				if (!internalClockSelect) {
					HandleNoteOff(sequence[seqCurrentStep], 0);
					midiClockRunning = false;
				}
			}
		}
	}
}

///////////////////////
// SEQUENCER
///////////////////////

void seqPlayStep(byte step) {
	if (sequence[step]) { // if the step is not a zero
		if (noteIsOn && seqLength > 0) {
			// turn off prev note (was (step-1)%16, which missed for seqLength != 16 —
			// mono didn't care, but a missed off would leave a poly voice hanging)
			HandleNoteOff(sequence[(byte)((step + seqLength - 1) % seqLength)], 0);
		}
		HandleNoteOn(sequence[step], 127); // turn on next note
		arcadeNote = sequence[step];
	}
}

void playNextStep() {
	seqCurrentStep++;
	seqCurrentStep = seqCurrentStep % seqLength;
	seqPlayStep(seqCurrentStep);
}

void handleSeqClock() { // handle internal sequencer clock
	seqIncrement++;

	if (seqIncrement > seqNoteLength && noteIsOn) { // turn note off after it reached its length
		HandleNoteOff(sequence[seqCurrentStep], 0);
	}

	if (seqIncrement > seqTempo) { // go to next step after prev step was nailored
		playNextStep();
		seqIncrement = 0;
	}
}

void handleSequencer() {
	if (internalClockSelect) {
		handleSeqClock();
	}
	// external clock mode: steps come from MIDI clock (callbacks above) or
	// manual stepping with the arcade button. The old LED-pin gate-sync input
	// is gone along with the LEDs.
	else if (midiClockRunning && millis() - lastMidiTickMs > 500) {
		// no tick for 500ms (real clock ticks every ~21ms at 120bpm): the
		// source is gone, or a stray tick at boot latched us here. Without
		// this, a single spurious tick leaves a note hanging forever and
		// permanently steals the arcade button's step function.
		midiClockRunning = false;
		HandleNoteOff(sequence[seqCurrentStep], 0);
	}
}

void setWriteNote(byte thisNote) {
	if (thisNote != noteSelect || refreshWriteNotePing) { // if we are on a new note
		octOffset = writeOctSelect * 12;
		noteSelect = thisNote;
		HandleNoteOff(prevNoteSelect, 0);
		noteToWrite = noteSelect + octOffset;
		HandleNoteOn(noteToWrite, 127);
		prevNoteSelect = noteSelect;
		refreshWriteNotePing = false;
	}
}

void writeToSeq() {
	sequence[seqCurrentStep] = noteToWrite;
	seqLength++; // extend sequence length
	seqLength = seqLength % sizeof(sequence); // don't allow sequence longer than the buffer
	seqCurrentStep++;
	seqCurrentStep = seqCurrentStep % sizeof(sequence);
}

void writeSeqToEeprom() {
	for (int i = 0; i < seqLength; i++) {
		EEPROM.write(EE_SEQ_BASE + i, sequence[i]);
	}
	EEPROM.write(EE_ADDR_LEN, seqLength);
	EEPROM.write(EE_ADDR_MAGIC, EE_MAGIC);
	EEPROM.commit(); // ESP32: actually persist to flash
}

void readSeqFromEeprom() {
	seqLength = EEPROM.read(EE_ADDR_LEN); // need length first so we know how far to read
	for (int i = 0; i < seqLength; i++) {
		sequence[i] = EEPROM.read(EE_SEQ_BASE + i);
	}
}

bool EEPROMwriting = false;    // flag to ignore all buttons until every button is released
void seqCheckButts() {
	if (EEPROMwriting) {
		if (!(buttStates[BTN_A] || buttStates[BTN_B] || buttStates[BTN_SHIFT])) {
			EEPROMwriting = false;
		}
	}
	else {

		// BTN_A

		if (buttStates[BTN_A] && !oldButtStates[BTN_A]) {
			oldButtStates[BTN_A] = buttStates[BTN_A];
			if (buttStates[BTN_B] && buttStates[BTN_SHIFT]) {
				writeSeqToEeprom();
				EEPROMwriting = true; // ignore all buttons until every button is released
			}
			else {
				internalClockSelect = !internalClockSelect;       // toggle clock source
				if (!internalClockSelect) {                       // landed on ext clock
					HandleNoteOff(sequence[seqCurrentStep], 127); // stop any note that might be on
					seqIncrement = seqTempo;                      // prime incrementor so it starts
				}
				else {                                            // landed on internal clock
					seqCurrentStep = seqLength - 1;
					seqIncrement = seqTempo;
				}
			}
		}
		else if (!buttStates[BTN_A] && oldButtStates[BTN_A]) {
			oldButtStates[BTN_A] = buttStates[BTN_A];
		}

		// BUTTON 2

		else if (buttStates[BTN_B] && !oldButtStates[BTN_B]) { // just pressed
			oldButtStates[BTN_B] = buttStates[BTN_B];
			if (buttStates[BTN_SHIFT]) {
				// don't do nuttin cos we are preparing for 3 button EEPROM WRITE
			}
			else {
				writeMode = true;
				seqLength = 0;      // reset sequence length
				seqCurrentStep = 0; // reset sequence
			}
		}
		else if (!buttStates[BTN_B] && oldButtStates[BTN_B]) { // write button released!!!
			seqCurrentStep = seqLength - 1;
			writeMode = false;
			HandleNoteOff(noteToWrite, 127);
			oldButtStates[BTN_B] = buttStates[BTN_B];
		}

		// BUTTON 3

		else if (buttStates[BTN_SHIFT] && !oldButtStates[BTN_SHIFT]) {
			oldButtStates[BTN_SHIFT] = buttStates[BTN_SHIFT];
			if (writeMode) {
				noteToWrite = 0;
				writeToSeq(); // write a pause
			}
		}
		else if (!buttStates[BTN_SHIFT] && oldButtStates[BTN_SHIFT]) {
			oldButtStates[BTN_SHIFT] = buttStates[BTN_SHIFT];
		}
	}
}

///////////////////////
// WAVEFORM SELECT
///////////////////////

// note: table sizes differ from the Oscil's 1024 cells (2048/512 tables play
// at half/double stride) — that's inherited from the original and part of the sound
const int8_t* waveTable(byte w) {
	switch (w & 3) {
	default:
	case 0: return SIN1024_DATA;
	case 1: return TRIANGLE2048_DATA;
	case 2: return SAW2048_DATA;
	case 3: return SQUARE_ANALOGUE512_DATA;
	}
}

void setWaveForm(byte waveNumber) {
	if (buttStates[BTN_SHIFT]) {
		modWave = waveNumber; // remember for the OLED
		for (int i = 0; i < NUM_VOICES; i++) voices[i].mod.setTable(waveTable(waveNumber));
	}
	else {
		carWave = waveNumber;
		for (int i = 0; i < NUM_VOICES; i++) voices[i].car.setTable(waveTable(waveNumber));
	}
}

///////////////////////
// PRESETS (SET page: A saves, B loads)
///////////////////////

void savePreset(byte slot) {
	Preset p;
	p.magic = PRESET_MAGIC;
	p.carWave = carWave;         p.modWave = modWave;
	p.lpfCutoff = lpfCutoff;     p.lpfRes = lpfRes;
	p.mod_ratio = (byte)mod_ratio;
	p.lfoDest = lfoDest;         p.lfoWaveSelect = lfoMode; // field reused as lfoMode
	p.bit7 = bit7Mode;
	p.env2FM = env2FM;           p.env2Filt = env2Filt;
	p.env2Ratio = env2Ratio;
	p.polyFilt = polyFilter;
	p.fmIntensity = fm_intensity;
	p.attack = dispAttack;       p.decay = dispDecay;
	p.sustain = dispSustain;     p.release = dispRelease;
	p.e2Attack = e2Attack;       p.e2Decay = e2Decay;
	p.e2Sustain = e2Sustain;     p.e2Release = e2Release;
	p.lfoRate = lfoRate;         p.modDepth = modDepth;
	p.rvSize = rvSize;           p.rvDamp = rvDamp;
	p.rvMix = rvMix;             p.rvSpread = rvSpread;
	p.oscVol = oscVol;           p.limThresh = limThresh;
	EEPROM.put(EE_PRESET_ADDR + slot * EE_PRESET_STRIDE, p);
	EEPROM.commit(); // flash write: expect a tiny audio hiccup, same as a seq save
	presetUsed[slot] = true;
}

bool loadPreset(byte slot) {
	Preset p;
	EEPROM.get(EE_PRESET_ADDR + slot * EE_PRESET_STRIDE, p);
	if (p.magic != PRESET_MAGIC) return false; // nothing saved in this slot yet

	carWave = p.carWave & 3;
	modWave = p.modWave & 3;
	dispAttack = p.attack;   dispDecay = p.decay;
	dispSustain = p.sustain; dispRelease = p.release;
	for (int i = 0; i < NUM_VOICES; i++) {
		voices[i].car.setTable(waveTable(carWave));
		voices[i].mod.setTable(waveTable(modWave));
		voices[i].env.setAttackTime(dispAttack);
		voices[i].env.setDecayTime(dispDecay);
		voices[i].env.setSustainLevel(dispSustain);
		voices[i].env.setDecayLevel(dispSustain);
		voices[i].env.setReleaseTime(dispRelease);
	}
	fm_intensity = p.fmIntensity;
	lpfCutoff = p.lpfCutoff;
	lpfRes = p.lpfRes;
	lpf.setResonance((uint16_t)lpfRes << 8); // cutoff is applied every tick anyway
	mod_ratio = p.mod_ratio;
	lfoDest = p.lfoDest;
	lfoMode = p.lfoWaveSelect; // field reused as lfoMode (old presets: 0/1 still valid)
	lfoRate = p.lfoRate;
	LFO.setFreq(lfoRate);
	modDepth = p.modDepth;
	offsetOn = (modDepth != 0);
	rvSize = p.rvSize; reverb.setRoomSize(rvSize);
	rvDamp = p.rvDamp; reverb.setDamp(rvDamp);
	rvMix = p.rvMix; // wet/dry gains are re-derived from this every control tick
	rvSpread = p.rvSpread;
	// presets saved before rvSpread existed carry garbage/NaN in this slot;
	// fall back to natural width so they still load cleanly
	if (!(rvSpread >= 0.0f && rvSpread <= 16.0f)) rvSpread = 1.0f;
	bit7Mode = p.bit7;
	env2FM = p.env2FM;       env2Filt = p.env2Filt;
	env2Ratio = p.env2Ratio;
	e2Attack = p.e2Attack;   e2Decay = p.e2Decay;
	e2Sustain = p.e2Sustain; e2Release = p.e2Release;
	polyFilter = p.polyFilt;
	for (int i = 0; i < NUM_VOICES; i++) {
		voices[i].env2.setAttackTime(e2Attack);
		voices[i].env2.setDecayTime(e2Decay);
		voices[i].env2.setSustainLevel(e2Sustain);
		voices[i].env2.setDecayLevel(e2Sustain);
		voices[i].env2.setReleaseTime(e2Release);
		voices[i].lpf.setResonance((uint16_t)lpfRes << 8);
	}
	oscVol = p.oscVol;
	limThresh = p.limThresh;

	lockKnobs(); // so the pots don't stomp the loaded values until deliberately moved
	return true;
}

void envCheckButts() { // ENV page: A toggles ENV1/ENV2, B toggles the mod-routing view
	if (buttStates[BTN_A] && !oldButtStates[BTN_A]) {
		envSelect = !envSelect;
		lockKnobs(); // knob positions belong to the other envelope
	}
	if (buttStates[BTN_B] && !oldButtStates[BTN_B]) {
		envRouteView = !envRouteView;
		lockKnobs(); // knobs mean something different in the routing view
	}
	oldButtStates[BTN_A] = buttStates[BTN_A];
	oldButtStates[BTN_B] = buttStates[BTN_B];
	// keep SHIFT's edge state in sync so a held button carried onto another
	// page can't fire a false edge there
	oldButtStates[BTN_SHIFT] = buttStates[BTN_SHIFT];
}

void setCheckButts() { // SET page buttons: preset save/load through a 10-slot selector
	if (buttStates[BTN_A] && !oldButtStates[BTN_A]) {
		if (presetSelMode == 1) {      // selector open in save mode: confirm
			savePreset(presetSelSlot);
			presetMsg = 1;
			presetMsgUntil = millis() + 1200;
			presetSelMode = 0;
			lockKnobs(); // knob 1 was scrolling slots — don't let it stomp 7BIT now
		}
		else if (presetSelMode == 2) { // other button while open = cancel
			presetSelMode = 0;
			lockKnobs();
		}
		else {
			presetSelMode = 1;         // open the save selector
		}
	}
	else if (buttStates[BTN_B] && !oldButtStates[BTN_B]) {
		if (presetSelMode == 2) {      // selector open in load mode: confirm
			presetMsg = loadPreset(presetSelSlot) ? 2 : 3;
			presetMsgUntil = millis() + 1200;
			presetSelMode = 0;
			lockKnobs(); // loadPreset locks too, but cover the "NO PRESET" path
		}
		else if (presetSelMode == 1) { // other button while open = cancel
			presetSelMode = 0;
			lockKnobs();
		}
		else {
			presetSelMode = 2;         // open the load selector
		}
	}
	oldButtStates[BTN_A] = buttStates[BTN_A];
	oldButtStates[BTN_B] = buttStates[BTN_B];
}

///////////////////////
// OLED (runs on core 0)
///////////////////////
// Arduino/Mozzi run on core 1; the SSD1306 framebuffer push over I2C takes
// ~23ms at 400kHz, so drawing lives in its own task pinned to core 0. It only
// reads the globals above — a torn read once in a while just means one frame
// shows a value a tick late, which doesn't matter at 15fps.

const char* PAGE_NAMES[6]  = { "MAIN", "AMPENV", "LFO", "VERB", "H4XX", "SEQ" };
const char* WAVE_NAMES[4]  = { "SIN", "TRI", "SAW", "SQR" };
const char* LFO_DESTS[4]   = { "PITCH", "FILTER", "FM", "-" };
const char* NOTE_NAMES[12] = { "C","C#","D","D#","E","F","F#","G","G#","A","A#","B" };

void noteName(byte n, char* buf, size_t len) {
	snprintf(buf, len, "%s%d", NOTE_NAMES[n % 12], n / 12 - 1);
}

// heartbeat spinner, bottom right: if it stops turning, this task has died
void drawSpinner() {
	static byte frame = 0;
	frame++;
	switch ((frame >> 2) & 3) { // advances every 4 frames (~every 260ms)
	case 0: oled.drawFastVLine(124, 58, 5, SSD1306_WHITE); break;             // |
	case 1: oled.drawLine(122, 62, 126, 58, SSD1306_WHITE); break;            // /
	case 2: oled.drawFastHLine(122, 60, 5, SSD1306_WHITE); break;             // -
	case 3: oled.drawLine(122, 58, 126, 62, SSD1306_WHITE); break;            // backslash
	}
}

void drawUI() {
	const char* lab[4];
	char val[4][16];
	byte page = pageState; // snapshot, core 1 may change it mid-draw

	// the preset slot selector takes over the whole screen while open
	if (page == PG_HAXX && presetSelMode) {
		byte mode = presetSelMode; // snapshot both, core 1 may change them mid-draw
		byte sel = presetSelSlot;
		oled.clearDisplay();
		oled.setTextSize(2);
		oled.setCursor(0, 0);
		oled.print(mode == 1 ? "SAVE TO" : "LOAD");
		oled.setTextSize(1);
		for (byte i = 0; i < NUM_PRESETS; i++) { // 2 rows x 5 slots; box = saved, filled = selected
			int bx = 4 + (i % 5) * 24;
			int by = 24 + (i / 5) * 17;
			if (i == sel) {
				oled.fillRect(bx, by, 20, 13, SSD1306_WHITE);
				oled.setTextColor(SSD1306_BLACK);
			}
			else if (presetUsed[i]) {
				oled.drawRect(bx, by, 20, 13, SSD1306_WHITE);
			}
			oled.setCursor(bx + (i == 9 ? 5 : 8), by + 3);
			oled.print(i + 1);
			oled.setTextColor(SSD1306_WHITE);
		}
		oled.setCursor(0, 56);
		oled.print(mode == 1 ? "K1:PICK A:SAVE PG:X" : "K1:PICK B:LOAD PG:X");
		drawSpinner();
		oled.display();
		return;
	}

	// one row per knob, top to bottom = knob 1..4 (empty label = unused knob, skipped)
	for (byte i = 0; i < 4; i++) { lab[i] = ""; val[i][0] = '\0'; }
	switch (page) {
	case PG_MAIN:
		lab[0] = "FM/RAT";  snprintf(val[0], 16, "%ld/%d", fm_intensity, mod_ratio);
		lab[1] = "CUT/RES"; snprintf(val[1], 16, "%u/%u", lpfCutoff, lpfRes);
		lab[2] = "WAVE";    snprintf(val[2], 16, "%s/%s", WAVE_NAMES[carWave & 3], WAVE_NAMES[modWave & 3]);
		lab[3] = "VOL/LIM"; snprintf(val[3], 16, "%.2f/%u%%", oscVol, (unsigned)(limThresh * (100.0f / 65536.0f)));
		break;
	case PG_ENV:
		if (envRouteView) { // ENV2 destination amounts (reverb send removed)
			lab[0] = "FM";     snprintf(val[0], 16, "%u", env2FM);
			lab[1] = "FILTER"; snprintf(val[1], 16, "%u", env2Filt);
			lab[2] = "RATIO";  snprintf(val[2], 16, "%u", env2Ratio);
		}
		else if (envSelect) { // ENV2 A/D/S/R
			lab[0] = "ATTACK";  snprintf(val[0], 16, "%d", e2Attack);
			lab[1] = "DECAY";   snprintf(val[1], 16, "%d", e2Decay);
			lab[2] = "SUSTAIN"; snprintf(val[2], 16, "%d", e2Sustain);
			lab[3] = "RELEASE"; snprintf(val[3], 16, "%d", e2Release);
		}
		else { // ENV1 A/D/S/R
			lab[0] = "ATTACK";  snprintf(val[0], 16, "%d", dispAttack);
			lab[1] = "DECAY";   snprintf(val[1], 16, "%d", dispDecay);
			lab[2] = "SUSTAIN"; snprintf(val[2], 16, "%d", dispSustain);
			lab[3] = "RELEASE"; snprintf(val[3], 16, "%d", dispRelease);
		}
		break;
	case PG_LFO:
		lab[0] = "RATE";  snprintf(val[0], 16, "%.2fHz", lfoRate);
		lab[1] = "DEPTH"; snprintf(val[1], 16, "%.2f", modDepth);
		lab[2] = "DEST";  snprintf(val[2], 16, "%s", LFO_DESTS[lfoDest & 3]);
		lab[3] = "WAVE";  snprintf(val[3], 16, "%s",
		                          lfoMode == 0 ? "MORPH" : lfoMode == 1 ? "RND"
		                        : lfoMode == 2 ? "STAT+" : lfoMode == 3 ? "STAT-" : "STAT+-");
		break;
	case PG_VERB:
		lab[0] = "SIZE";   snprintf(val[0], 16, "%.2f", rvSize);
		lab[1] = "DAMP";   snprintf(val[1], 16, "%.2f", rvDamp);
		lab[2] = "MIX";    snprintf(val[2], 16, "%.2f", rvMix);
		lab[3] = "SPREAD"; snprintf(val[3], 16, "%.2f", rvSpread);
		break;
	case PG_HAXX:
		lab[0] = "7BIT";   snprintf(val[0], 16, "%s", bit7Mode ? "ON" : "OFF");
		lab[3] = "FILTER"; snprintf(val[3], 16, "%s", polyFilter ? "POLY" : "PARA");
		break;
	case PG_SEQ:
		if (internalClockSelect) {
			lab[0] = "TEMPO"; snprintf(val[0], 16, "%u", seqTempo);
			lab[1] = "GATE";  snprintf(val[1], 16, "%u", seqNoteLength);
		}
		else {
			lab[0] = "CLK STEP"; snprintf(val[0], 16, "%u ticks", midiClockStepSize);
			lab[1] = "GATE";     snprintf(val[1], 16, "%u", midiSeqNoteLength);
		}
		lab[2] = "NOTE";
		if (writeMode) noteName(noteToWrite, val[2], 16);
		else           snprintf(val[2], 16, "-");
		lab[3] = "OCT"; snprintf(val[3], 16, "%+d", octTranspose);
		break;
	}

	oled.clearDisplay();

	// header: page name + 6-dot page indicator
	oled.setTextSize(2);
	oled.setCursor(0, 0);
	// the ENV page titles itself by which envelope the knobs are editing:
	// "ENV" with a small amp/mod tag, or just "MOD" in the routing view
	if (page == PG_ENV) {
		if (envRouteView) {
			oled.print("MOD");
		}
		else {
			oled.print("ENV");
			oled.setTextSize(1);
			oled.setCursor(38, 9); // small tag after the big ENV, bottom-aligned
			oled.print(envSelect ? "mod" : "amp");
			oled.setTextSize(2);
		}
	}
	else oled.print(PAGE_NAMES[page]);
	for (byte i = 0; i < 6; i++) {
		if (i == page) oled.fillRect(74 + i * 9, 4, 6, 6, SSD1306_WHITE);
		else           oled.drawRect(74 + i * 9, 4, 6, 6, SSD1306_WHITE);
	}

	oled.setTextSize(1);
	for (byte i = 0; i < 4; i++) {
		if (lab[i][0] == '\0') continue; // knob unused on this page, leave its row blank
		int y = 16 + i * 10;
		oled.setCursor(0, y);
		oled.print(lab[i]);
		oled.setCursor(64, y);
		if (knobLock[i]) oled.print('*'); // knob not picked up since page change
		oled.print(val[i]);
	}

	// footer on the SEQ page: clock source, write/play state, position
	if (page == PG_SEQ) {
		char foot[24];
		snprintf(foot, 24, "%s %s %u/%u",
		         internalClockSelect ? "INT" : (midiClockRunning ? "EXT>" : "EXT"),
		         writeMode ? "REC" : (noteIsOn ? ">" : " "),
		         seqCurrentStep + 1, seqLength);
		oled.setCursor(0, 56);
		oled.print(foot);
	}

	// footer on the ENV page: which envelope / mode the knobs are editing
	if (page == PG_ENV) {
		oled.setCursor(0, 56);
		oled.print(envRouteView ? "MOD ROUTE   B:BACK"
		                        : "A:E1/E2  B:MOD");
	}

	// footer on the LFO page: A cycles the wave; knob 4's job depends on the mode
	if (page == PG_LFO) {
		oled.setCursor(0, 56);
		oled.print(lfoMode == 0 ? "A:WAVE  K4:MORPH" : "A:WAVE  K4:SLEW");
	}

	// footer on the H4XX page: preset button hint / save-load confirmation
	if (page == PG_HAXX) {
		oled.setCursor(0, 56);
		if (millis() < presetMsgUntil) {
			oled.print(presetMsg == 1 ? "SAVED"
			         : presetMsg == 2 ? "LOADED"
			         : presetMsg == 4 ? "SYSEX RX"
			                          : "NO PRESET");
		}
		else {
			oled.print("A:SAVE  B:LOAD");
		}
	}

	drawSpinner();
	oled.display();
}

// boot splash: the gateXtal wordmark with a little play-triangle glyph
void drawSplash() {
	oled.clearDisplay();
	oled.setTextSize(2);
	oled.setTextColor(SSD1306_WHITE);
	oled.setCursor(6, 26);
	oled.print(F("gateXtal"));
	oled.drawLine(122, 33, 104, 33, SSD1306_WHITE);
	oled.drawTriangle(104, 40, 113, 24, 121, 40, SSD1306_WHITE);
	oled.setTextSize(1);
  	oled.setTextColor(SSD1306_WHITE);
  	oled.setCursor(107, 45);
  	oled.print(F("FM"));
  	oled.drawLine(107, 52, 107, 56, SSD1306_WHITE);
  	oled.drawLine(113, 51, 113, 53, SSD1306_WHITE);
  	oled.drawLine(117, 52, 117, 57, SSD1306_WHITE);
  	oled.drawLine(115, 50, 115, 51, SSD1306_WHITE);
	oled.display();
}

void displayTask(void*) {
	Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
	Wire.setClock(400000);
	if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
		vTaskDelete(NULL); // no display found, don't keep hammering the bus
	}
	oled.setTextColor(SSD1306_WHITE);
	oled.setTextWrap(false);
	drawSplash();
	vTaskDelay(pdMS_TO_TICKS(1500)); // hold the splash before the UI takes over
	for (;;) {
		drawUI();
		vTaskDelay(pdMS_TO_TICKS(66)); // ~15 fps
	}
}

///////////////////////
// SETUP
///////////////////////

void lockKnobs() {
	for (int i = 0; i < 4; i++) {
		lockAnchor[i] = mozziRaw[i];
		knobLock[i] = true;
	}
}

void setup() {
	// quick white blink so we can see the firmware booted at all
#ifdef RGB_BUILTIN
	rgbLedWrite(RGB_BUILTIN, RGB_BRIGHTNESS, RGB_BRIGHTNESS, RGB_BRIGHTNESS);
	delay(150);
	rgbLedWrite(RGB_BUILTIN, 0, 0, 0);
#endif

	EEPROM.begin(EEPROM_SIZE);
	if (EEPROM.read(EE_ADDR_MAGIC) == EE_MAGIC) {
		readSeqFromEeprom();
	}
	for (byte i = 0; i < NUM_PRESETS; i++) { // scan which slots hold a patch, for the selector UI
		presetUsed[i] = (EEPROM.read(EE_PRESET_ADDR + i * EE_PRESET_STRIDE) == PRESET_MAGIC);
	}

	for (int i = 0; i < 4; i++) {
		pinMode(BUTTONS[i], INPUT_PULLUP);
	}
	pinMode(PIN_ARCADE, INPUT_PULLUP);

	// USB: enumerate as a MIDI device (plus a CDC serial port for debugging
	// and upload auto-reset). VID/PID/name must be set before USB.begin().
	USB.VID(0x1209);  // pid.codes open-source hardware VID
	USB.PID(0x2020);  // our allocated PID
	USB.manufacturerName("Captain Credible");
	USB.productName("GateXtal");
	// unique per-unit serial from the chip's eFuse MAC, so the OS can tell two
	// GateXtals apart (setters copy the string, so a local buffer is fine)
	char serial[13];
	uint64_t mac = ESP.getEfuseMac();
	snprintf(serial, sizeof(serial), "%04X%08X", (uint16_t)(mac >> 32), (uint32_t)mac);
	USB.serialNumber(serial);
	usbMIDI.begin();
	SerialCDC.begin();
	USB.begin();

	MIDI.setHandleNoteOn(HandleDINNoteOn);
	MIDI.setHandleNoteOff(HandleDINNoteOff);
	MIDI.setHandleStart(handleMIDIClockStart);
	MIDI.setHandleStop(handleMIDIClockStop);
	MIDI.setHandleClock(handleMIDIClock);
	MIDI.setHandleSystemExclusive(HandleDINSysEx); // bootloader-entry SysEx on DIN too
	MIDI.begin(MIDI_CHANNEL_OMNI); // listen to all channels
	MIDI.turnThruOff();            // no TX pin attached anyway
	// re-init UART1 with our RX pin; TX = -1 keeps GPIO43 free for the arcade button
	Serial1.begin(31250, SERIAL_8N1, PIN_MIDI_RX, -1);

	for (int i = 0; i < NUM_VOICES; i++) {
		voices[i].env.setADLevels(255, 240);
		voices[i].env.setTimes(100, 200, 65000, 200); // 65000 so the note sustains 65 seconds unless a noteOff comes
		voices[i].env2.setADLevels(255, 240);
		voices[i].env2.setTimes(100, 200, 65000, 200); // mod envelope, same defaults as the amp env
		voices[i].car.setFreq(440); // default frequency
		voices[i].lpf.setResonance(200 << 8); // per-voice filters for POLY mode
		voices[i].lpf.setCutoffFreq(100 << 8);
	}
	reverb.setWet(1.0f); // wet level is handled by input scaling (wetBase + per-voice sends)
	lpf.setResonance(200 << 8);
	lpf.setCutoffFreq(100 << 8);
	LFO.setFreq(1.f);

	// boot into preset slot 1 (index 0) if it holds a patch; no-op if empty.
	// after all the default object setup above, so it overrides those defaults.
	loadPreset(0);

	// OLED on core 0 — all I2C traffic stays off the audio core.
	// 8k stack: snprintf("%f") through newlib can chew a surprising amount.
	xTaskCreatePinnedToCore(displayTask, "oled", 8192, NULL, 1, NULL, 0);

	startMozzi();
}

///////////////////////
// CONTROL
///////////////////////

void updateControl() {

	// heartbeat: dim white breathing when idle, so we can tell the control
	// loop on core 1 is alive (the per-note color overrides it while held)
#ifdef RGB_BUILTIN
	static uint16_t hbCounter = 0;
	hbCounter++;                              // 512-tick cycle = 2s at 256Hz
	if (!noteIsOn && (hbCounter & 7) == 0) {  // refresh every 8th tick (32Hz)
		byte phase = (hbCounter >> 3) & 63;                // 0..63
		byte tri = (phase < 32) ? phase : (63 - phase);    // triangle 0..31..0
		byte w = tri >> 2;                                 // max ~7/255, subtle
		rgbLedWrite(RGB_BUILTIN, w, w, w);                 // white breathe
	}
#endif

	usbmidiprocessing();     // check for USB midi (drains all pending packets)
	while (MIDI.read()) {}   // check for DIN midi — drain everything pending, one
	                         // read() only parses a single message

	// pots are scanned on alternate ticks: 128Hz knob response as before, without
	// doubling the ADC burst load now that the control rate is 256Hz
	static bool potPhase = false;
	potPhase = !potPhase;
	if (potPhase)
	for (int i = 0; i < 4; i++) {
		mozziAnalogRead<10>(KNOBS[i]);         // throwaway read: the ADC's sample cap still
		                                       // holds the previous channel's voltage
		// burst of 5, drop min and max, average the middle 3 (10-bit range like the
		// old AVR ADC). A trimmed mean is robust against single-sample impulses,
		// which the adaptive filter below would otherwise pass through as "real".
		int mn = 1024, mx = -1, sum = 0;
		for (int j = 0; j < 5; j++) {
			int r = mozziAnalogRead<10>(KNOBS[i]);
			sum += r;
			if (r < mn) mn = r;
			if (r > mx) mx = r;
		}
		int v = (sum - mn - mx) / 3;

		// adaptive smoothing: filter strength scales with how fast the value is moving.
		// Noise-sized deltas get squashed hard, a real turn passes ~1:1, so it reads
		// steady at rest with no perceptible lag while turning. (At boot the huge first
		// delta makes it snap straight to the real position.)
		float delta = (float)v - knobSmooth[i];
		float a = fabsf(delta) * (1.0f / 40.0f); // full speed once the move is >= 40 counts
		if (a > 1.0f) a = 1.0f;
		if (a < 0.02f) a = 0.02f;                // but always converge, however slowly
		knobSmooth[i] += delta * a;

		// edge clamp: the S3 ADC is at its worst near the rails and real pots rarely
		// reach a true 0/1023, so stretch 8..1015 to full range and saturate
		int scaled = constrain((int)map((long)(knobSmooth[i] + 0.5f), 8, 1015, 0, 1023), 0, 1023);

		// 2-count output hysteresis so a value resting on a rounding boundary can't
		// flip-flop. Unlike the old deadband this compares against the *output* and
		// never re-anchors on raw reads, so noise can't slowly walk through it.
		// The ends are exempt so the pot can always land on exactly 0 and 1023.
		if (abs(scaled - mozziRaw[i]) >= 2 || (scaled != mozziRaw[i] && (scaled == 0 || scaled == 1023))) {
			mozziRaw[i] = scaled;
		}

		if (knobLock[i]) { // if this knob is locked
			if (mozziRaw[i] > lockAnchor[i] + lockThresh || mozziRaw[i] < lockAnchor[i] - lockThresh) {
				knobLock[i] = false; // wandered far enough from the anchor, start listening
			}
		}
	}

	for (int i = 0; i < 4; i++) {
		buttStates[i] = !digitalRead(BUTTONS[i]); // get buttstates
	}
	ArcadeState = !digitalRead(PIN_ARCADE);

	// HANDLE PAGE BUTTON (single button now, cycles through all 6 pages)
	if (buttStates[BTN_PAGE] && !oldButtStates[BTN_PAGE]) {
		if (presetSelMode) {
			presetSelMode = 0; // escape hatch: close the preset selector, stay on SET
			lockKnobs();
		}
		else {
			lockKnobs();
			pageState = (pageState + 1) % PG_COUNT;
		}
		oldButtStates[BTN_PAGE] = buttStates[BTN_PAGE];
	}
	else if (!buttStates[BTN_PAGE] && oldButtStates[BTN_PAGE]) {
		oldButtStates[BTN_PAGE] = buttStates[BTN_PAGE];
	}

	//////////////////////////////
	//// HANDLE ARCADE BUTTON ////
	//////////////////////////////

	if (ArcadeState && !oldArcadeState) { // if Arcadebutton is Pressed

		if (pageState == PG_SEQ && buttStates[BTN_B]) {
			writeToSeq();
		}
		else if (!internalClockSelect && !midiClockRunning) {
			playNextStep();
		}
		else {
			seqCurrentStep = seqLength - 1; // move to the last step
			seqIncrement = seqTempo;        // prime incrementor to roll over and thus trigger first step
			midiClockTicks = 0;
		}

		oldArcadeState = ArcadeState;
	}
	else if (!ArcadeState && oldArcadeState) {
		if (!writeMode) {
			HandleNoteOff(arcadeNote, 0);
		}
		oldArcadeState = ArcadeState;
	}

	//////////////////////
	// HANDLE FM KNOB   //  1
	//////////////////////

	if (presetSelMode == 0 && mozziRaw[FMknob] != oldMozziRaw[FMknob] && !knobLock[FMknob]) {

		int val = mozziRaw[FMknob];

		switch (pageState) {
		case PG_MAIN:
			if (buttStates[BTN_SHIFT]) mod_ratio = (val >> 6); // SHIFT: RATIO
			else                       fm_intensity = (float(val));
			break;
		case PG_ENV:
			if (envRouteView) {
				env2FM = val >> 2; // ENV2 -> FM routing amount
			}
			else if (envSelect) {
				e2Attack = val + 8;
				for (int vi = 0; vi < NUM_VOICES; vi++) voices[vi].env2.setAttackTime(e2Attack);
			}
			else {
				dispAttack = val + 8;
				for (int vi = 0; vi < NUM_VOICES; vi++) voices[vi].env.setAttackTime(dispAttack);
			}
			break;
		case PG_LFO:
			rndFreq = (val * -1 + 1024) << 5; // invert and scale down

			freeq = val;
			lfoRate = freeq / 50;
			LFO.setFreq(lfoRate); // CONTROL LFO RATE
			break;
		case PG_VERB:
			rvSize = val / 1023.0f;
			reverb.setRoomSize(rvSize);
			break;
		case PG_HAXX:
			bit7Mode = (val >= 512); // knob as a switch: right half = crunch
			break;
		case PG_SEQ:
			if (internalClockSelect) {
				seqTempo = (val >> 3) + 3;        // SCALE DOWN
				seqTempo = (seqTempo * -1) + 130; // INVERT
				seqTempo *= 2;                    // CONTROL_RATE 256: ticks are half as long
			}
			else {
				byte divisor = (val >> 7) % 4; // divisor is 01230123

				int tempVal = val - 512; // scale around 0
				if (tempVal <= 0) {
					midiClockStepSize = 24 >> divisor;
				}
				else {
					midiClockStepSize = 16 >> divisor;
				}
			}
			break;
		}

		oldMozziRaw[FMknob] = mozziRaw[FMknob];
	}

	////////////////////////
	// HANDLE KNOB 2      // (FILTER)
	////////////////////////

	if (presetSelMode == 0 && mozziRaw[h4xxKnob] != oldMozziRaw[h4xxKnob] && !knobLock[h4xxKnob]) { // if h4xxknob was moved
		int val = mozziRaw[h4xxKnob];

		switch (pageState) {
		case PG_MAIN:
			if (buttStates[BTN_SHIFT]) { // SHIFT: RES
				lpfRes = val >> 2;
				lpf.setResonance((uint16_t)lpfRes << 8);
				for (int vi = 0; vi < NUM_VOICES; vi++) voices[vi].lpf.setResonance((uint16_t)lpfRes << 8);
			}
			else {                       // CUT (applied every tick by the filter block below,
				lpfCutoff = val >> 2;    // which also folds in the LFO modulation)
			}
			break;
		case PG_ENV:
			if (envRouteView) {
				env2Filt = val >> 2; // ENV2 -> filter routing amount
			}
			else if (envSelect) {
				e2Decay = val;
				for (int vi = 0; vi < NUM_VOICES; vi++) voices[vi].env2.setDecayTime(val);
			}
			else {
				dispDecay = val;
				for (int vi = 0; vi < NUM_VOICES; vi++) voices[vi].env.setDecayTime(val);
			}
			break;
		case PG_LFO:
			// CONTROL LFO DEPTH BIPOLAR M8
			freeq = val;
			modDepth = freeq / 1000;
			offsetOn = modDepth;
			break;
		case PG_VERB:
			rvDamp = val / 1023.0f;
			reverb.setDamp(rvDamp);
			break;
		// PG_HAXX knob 2 is now unused (OSC VOL moved to MAIN knob 4)
		case PG_SEQ:
			seqNoteLength = map(val, 0, 1024, 0, seqTempo);
			midiSeqNoteLength = val >> 5; // scale val 0-32
			break;
		}
		oldMozziRaw[h4xxKnob] = mozziRaw[h4xxKnob];
	}

	//////////////////////
	// HANDLE KNOB 3    // (MOD OCT)
	//////////////////////
	if (presetSelMode == 0 && mozziRaw[attackKnob] != oldMozziRaw[attackKnob] && !knobLock[attackKnob]) {
		int val = mozziRaw[attackKnob];

		switch (pageState) { // knob does different things depending on pagestate
		case PG_MAIN: {
			byte w = val >> 8; // 0..3 waveform index
			byte cur = buttStates[BTN_SHIFT] ? modWave : carWave;
			if (w != cur) setWaveForm(w); // SHIFT: modulator wave, else carrier
			break;
		}
		case PG_ENV:
			if (envRouteView) {
				env2Ratio = val >> 2; // ENV2 -> ratio routing amount
			}
			else if (envSelect) {
				e2Sustain = val;
				for (int vi = 0; vi < NUM_VOICES; vi++) {
					voices[vi].env2.setSustainLevel(val);
					voices[vi].env2.setDecayLevel(val);
				}
			}
			else {
				dispSustain = val;
				for (int vi = 0; vi < NUM_VOICES; vi++) {
					voices[vi].env.setSustainLevel(val);
					voices[vi].env.setDecayLevel(val);
				}
			}
			break;
		case PG_LFO:
			// CONTROL LFO DEST
			lfoDest = val >> 8; // 4 different destinations
			break;
		case PG_VERB:
			rvMix = val / 1023.0f; // single wet/dry crossfade (applied every tick in the CV block)
			break;
		// PG_HAXX knob 3 is now unused (LIM THR moved to MAIN knob 4)
		case PG_SEQ:
			if (buttStates[BTN_B]) {
				setWriteNote(val >> 6);
			}
			break;
		default:
			break;
		}
		oldMozziRaw[attackKnob] = mozziRaw[attackKnob];
	}

	///////////////////////
	// HANDLE KNOB 4     // (waveform)
	///////////////////////
	if (presetSelMode == 0 && mozziRaw[releaseKnob] != oldMozziRaw[releaseKnob] && !knobLock[releaseKnob]) {
		int val = mozziRaw[releaseKnob];
		switch (pageState) { // knob does different things depending on pagestate
		case PG_MAIN:
			if (buttStates[BTN_SHIFT]) limThresh = 6554.0f + val * 57.7f; // SHIFT: LIM (~10..100%)
			else                       oscVol = val / 512.0f;             // VOL 0..2, unity at noon
			break;
		case PG_ENV:
			if (!envRouteView) { // routing view has no knob-4 destination anymore
				if (envSelect) {
					e2Release = val;
					for (int vi = 0; vi < NUM_VOICES; vi++) voices[vi].env2.setReleaseTime(val);
				}
				else {
					dispRelease = val;
					for (int vi = 0; vi < NUM_VOICES; vi++) voices[vi].env.setReleaseTime(val);
				}
			}
			break;
		// PG_LFO knob 4 is read live in the CV block below: it morphs the wave
		// in MORPH mode and sets the slew in RND / STAT mode. A picks the mode.
		case PG_VERB: {
			// quadratic: fine control down low (mono..natural sits in the first
			// third of travel), then it blows out to silly-wide at the top
			float s = val / 1023.0f;
			rvSpread = s * s * 8.0f; // 0 (mono) .. ~1 natural (~1/3 up) .. 8 (absurd)
			break;
		}
		case PG_HAXX:
			polyFilter = (val >= 512); // left = PARA (one filter), right = POLY (filter per voice)
			break;
		case PG_SEQ:
			if (val >> 7 != writeOctSelect) {
				if (buttStates[BTN_B] && writeMode) {
					writeOctSelect = val >> 7; // 0-8
					refreshWriteNotePing = true;
					setWriteNote(noteSelect);
				}
				else if (buttStates[BTN_B]) {
				}
				else { // if knob is twiddled and no butts are true
					octTranspose = val >> 7; // 0-8
					writeOctSelect = octTranspose;
					octTranspose = octTranspose - 4;
					applyOctTranspose(); // slide held voices without retrigging their ADSRs
				}
			}
			break;
		default:
			break;
		}
		oldMozziRaw[releaseKnob] = mozziRaw[releaseKnob];
	}

	// page-specific button handling
	if (pageState == PG_ENV) { // ENV page: A = ENV1/ENV2, B = routing view
		envCheckButts();
	}
	else if (pageState == PG_LFO) { // LFO page: A cycles the wave
		if (buttStates[BTN_A] && !oldButtStates[BTN_A]) {
			lfoMode = (lfoMode + 1) % 5; // MORPH, RND, STAT+, STAT-, STAT+-
		}
		oldButtStates[BTN_A] = buttStates[BTN_A];
		oldButtStates[BTN_B] = buttStates[BTN_B];
		oldButtStates[BTN_SHIFT] = buttStates[BTN_SHIFT];
	}
	else if (pageState == PG_SEQ) {
		seqCheckButts();
	}
	else if (pageState == PG_HAXX) { // H4XX page: preset save/load buttons + slot selector
		if (presetSelMode) {
			presetSelSlot = constrain(mozziRaw[FMknob] / 103, 0, NUM_PRESETS - 1); // knob 1 scrolls
		}
		setCheckButts();
	}

	//////////////////    /////			   /////
	//////////////////    /////			  /////
	/////				  /////			 /////
	/////				   /////        /////
	/////				   /////       /////
	/////					/////     /////
	/////					 /////   /////
	/////					  ///// /////
	//////////////////		   /////////
	//////////////////			//////

	// HANDLE INTERNAL "CV"

	if (offsetOn) {
		if (lfoMode == 0) {
			// MORPH: knob 4 morphs the shape — sine (far left) to falling saw
			// (far right). The saw is read straight off the LFO's own phase so
			// it stays locked to the sine.
			int8_t sn = LFO.next();
			int cell = (int)((LFO.getPhaseFractional() >> OSCIL_F_BITS) & (SIN512_NUM_CELLS - 1));
			int fall = 127 - (cell >> 1);             // 0..511 -> 127..-128 falling saw
			int k = mozziRaw[releaseKnob];            // knob 4: 0(sine)..1023(saw)
			int out = ((1023 - k) * (int)sn + k * fall) / 1023;
			lfoOutput = constrain(out, -128, 127);
		}
		else if (lfoMode == 1) {
			// RND: slewed random. Rate (knob 1) sets step rate, knob 4 sets slew.
			if (mozziMicros() - rndTimer > (unsigned long)(rndFreq << 3)) {
				int lfoOutputBUFFER = rand(0, 244); // if depth adjusts this we can skip a float calc later
				HDlfoOutputBuffer = Q16n0_to_Q16n16(lfoOutputBUFFER);
				rndTimer = mozziMicros();
			}
			// knob 4 = slew. Quadratic so the snappy end gets most of the travel
			// (linear wasted ~3/4 of the knob in the "fully smoothed" zone).
			int inv = 1023 - mozziRaw[releaseKnob];   // 0 (snappy) .. 1023 (heavy)
			Q16n16 slew = 2 + ((inv * inv) >> 12);    // ~2..257 slew steps at 256Hz
			aInterpolate.set(HDlfoOutputBuffer, slew);
			Q16n16 interpolatedLfoOutputBUFFER = aInterpolate.next();
			lfoOutput = Q16n16_to_Q16n0(interpolatedLfoOutputBUFFER) - 128; // scaled back down and offset to -128..128
		}
		else {
			// STAT: silence with random pops/crackle. Rate (knob 1, held in freeq)
			// sets pop density; knob 4 sets how fast each pop decays back to centre.
			// lfoMode 2 = STAT+ (up only), 3 = STAT- (down only), 4 = STAT+- (both).
			if ((int)rand(0, 1024) < ((int)freeq >> 1)) {
				if (lfoMode == 2)      statVal = rand(128, 256); // pop up:   out 0..127
				else if (lfoMode == 3) statVal = rand(0, 129);   // pop down: out -128..0
				else                   statVal = rand(0, 256);   // bipolar
			}
			else {                                     // otherwise decay toward centre (silence)
				int step = (mozziRaw[releaseKnob] >> 5) + 1; // knob 4: 1(slow tails)..32(snappy)
				if (statVal > 128) statVal = statVal - step < 128 ? 128 : statVal - step;
				else if (statVal < 128) statVal = statVal + step > 128 ? 128 : statVal + step;
			}
			lfoOutput = statVal - 128; // -128..127, 0 when idle
		}

	}

	// reverb base mix from the rvMix crossfade: dry full up to centre, then hands
	// over to wet (VERB page MIX knob).
	if (rvMix <= 0.5f) { wetBase = rvMix * 2.0f; dryNow = 1.0f; }
	else               { wetBase = 1.0f;         dryNow = (1.0f - rvMix) * 2.0f; }

	// base filter cutoff shared by both filter modes: knob + LFO (dest 1).
	// ENV2->filter is added per voice below.
	int cutBase = lpfCutoff;
	if (offsetOn && lfoDest == 1) {
		cutBase += (int)(lfoOutput * modDepth); // scale the LFO swing by depth (bipolar)
	}
	const int cutFloor = 4; // keep the filter just above fully-closed (no dead silence)

	// base FM shared by every voice: knob + LFO (dest 2). ENV2->FM is added per
	// voice below. The LFO sums on top of the FM knob rather than replacing it.
	long fmBase = fm_intensity;
	if (offsetOn && lfoDest == 2) {
		fmBase += (long)(lfoOutput * modDepth); // bipolar swing around the knob
		if (fmBase < 0) fmBase = 0;             // FM amount can't go negative
	}

	// per voice: both envelopes, ENV2 routing fan-out, shared-LFO vibrato, ratio tracking
	int e2Max = 0;
	for (int vi = 0; vi < NUM_VOICES; vi++) {
		Voice& v = voices[vi];
		v.env.update();
		v.env2.update();
		int e2v = v.env2.next(); // this voice's ENV2, 0..255
		if (e2v > e2Max) e2Max = e2v;
		v.fmNow = fmBase + (((long)env2FM * e2v) >> 6);  // ENV2 -> FM, per voice
		if (!v.active && !v.env.playing()) continue; // idle voice, skip the tuning maths
		if (polyFilter) { // POLY: this voice's own filter gets base + its own ENV2 push
			int c = constrain(cutBase + (((int)env2Filt * e2v) >> 8), cutFloor, 254);
			v.lpf.setCutoffFreq((uint16_t)c << 8);
		}
		int ratioNow = mod_ratio + (int)(((long)env2Ratio * e2v) >> 13); // ENV2 -> ratio, per voice
		if (ratioNow > 15) ratioNow = 15;
		float f = v.freq;
		if (offsetOn && lfoDest == 0) {
			f += lfoOutput * modDepth; // the one LFO wobbles every voice in step
		}
		v.car.setFreq(f);
		int mf;
		if (ratioNow < 8) {
			mf = (int)f >> (8 - ratioNow);
		}
		else {
			mf = (int)f * (ratioNow - 7);
		}
		v.mod.setFreq(mf);
	}
	if (!polyFilter) { // PARA: one filter on the sum; ENV2->filter follows the loudest ENV2
		int c = constrain(cutBase + (((int)env2Filt * e2Max) >> 8), cutFloor, 254);
		lpf.setCutoffFreq((uint16_t)c << 8);
	}
	handleSequencer();
}

///////////////////////
// AUDIO
///////////////////////

// tanh-shaped saturator (Pade approximation): no corner anywhere, just a
// progressively harder harmonic bend. Nearly linear below half scale, the rail
// lands at x=1.33 (~0.87 out), and it only fully flattens at 3x over. Returns
// roughly -1..1; the caller scales it back up to the output range.
static inline float softClip(float m) {
	float x = m * (1.0f / 49152.0f);
	if (x > 3.0f) x = 3.0f; else if (x < -3.0f) x = -3.0f;
	float xx = x * x;
	return x * (27.0f + xx) / (27.0f + 9.0f * xx); // == tanh to within 1%, exactly ±1 at ±3
}

AudioOutput updateAudio() {
	int32_t mix = 0;
	for (int vi = 0; vi < NUM_VOICES; vi++) {
		Voice& v = voices[vi];
		if (!v.active && !v.env.playing()) continue; // silent voice, save the cycles
		long modulation = v.fmNow * v.mod.next();    // per-voice FM depth (ENV2 routing)
		int e = v.env.next();
		int s = e * v.car.phMod(modulation); // ±32385 per voice, ~15 bits
		if (polyFilter) s = v.lpf.next(s);   // POLY: filter each voice before the sum
		mix += s;
	}
	mix = (int32_t)(mix * oscVol); // H4XX page OSC VOL: >1.0 drives the output stage

	int filtered = polyFilter ? mix : lpf.next(mix); // PARA: one filter on the sum, as ever
	if (bit7Mode) {
		filtered = (filtered >> 9) << 9; // crush to 128 levels — the old AVR resolution, reverb tail included
	}
	// reverb input scaled by the wet/dry crossfade base (wet_ inside is fixed at 1).
	// dry stays mono/centred; the stereo width lives entirely in the wet tail.
	float wetL, wetR;
	reverb.processStereo((float)filtered * wetBase, wetL, wetR);
	// SPREAD: mid/side width on the wet tail. rvSpread 0 = mono (both = mid),
	// 1 = the reverb's natural stereo, >1 = exaggerated. Dry stays centred.
	float mid = (wetL + wetR) * 0.5f;
	float side = (wetL - wetR) * 0.5f * rvSpread;
	float dry = (float)filtered * dryNow;
	float mL = dry + mid + side;
	float mR = dry + mid - side;

	// peak limiter: instant attack, ~100ms release. Ducks by exactly the
	// overshoot, so chords come down transparently instead of flat-topping.
	// Drive it from the louder channel so the stereo image isn't skewed.
	float level = fmaxf(fabsf(mL), fabsf(mR));
	if (level > limEnv) limEnv = level;
	else                limEnv *= LIM_RELEASE;
	if (limEnv > limThresh) { float g = limThresh / limEnv; mL *= g; mR *= g; }

	return StereoOutput::fromNBit(17, (int)(softClip(mL) * 65535.0f),
	                                 (int)(softClip(mR) * 65535.0f));
}

void loop() {
	audioHook(); // required here
}
