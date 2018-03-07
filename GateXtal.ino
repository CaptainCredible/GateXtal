
#define debugPort CompositeSerial
const int debugTogglePin = PA3;
bool debugToggle = true;

/*
  Name:		GateXtal.ino
  Created:	10/16/2017 9:18:53 AM
  Author:	Daniel Lacey

  TODO
  SEQ needs to trigger first note when turned on
*/

//LIBRARIES
#include <USBMIDI.h>
#include <USBCompositeSerial.h>
#include <EEPROM.h>
#include <MIDI.h>
#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <mozzi_midi.h>
#include <ADSR.h>
//#include <AutoMap.h> // maps npredictable inputs to a range
#include <LowPassFilter.h>
#include <mozzi_rand.h>
#include <Line.h>
#include <IntMap.h>
//const IntMap invert(0, 1024, 1024, 0);

//TABLES
//#include <tables/cos512_int8.h> // sine table for oscillator
//#include <tables/cos2048_int8.h> // sine table for oscillator
//#include <tables/sin2048_int8.h> // cosine table for oscillator
#include <tables/sin1024_int8.h> // cosine table for oscillator
#include <tables/saw2048_int8.h> // saw table for oscillator
#include <tables/triangle2048_int8.h> // triangle table for oscillator
#include <tables/square_analogue512_int8.h> // square table for oscillator
#include <tables/sin512_int8.h> //lofi sine for LFO
//#include <tables/brownnoise8192_int8.h> // recorded audio wavetable

//ERIC

int transpose = 0;
int octTranspose = 0;

byte midiClockStepSize = 24;

bool midiClockRunning = false;
bool writeMode = false;
bool ignoreLEDstate = false;
byte noteToWrite = 0;
byte octOffset = 0;
byte sequence[256] = { 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51, 40, 44, 47, 40, 44, 48, 40, 44, 47, 40, 44, 48, 40, 44, 47, 51 };
unsigned int seqIncrement = 0; //counter to keep track of when next step should come
byte seqCurrentStep = 0;
byte midiClockTicks = 0;
byte midiSeqNoteLength = 23;
byte seqNoteLength = 60;
byte midiClockDivider = 1;
bool internalClockSelect = false; //0 = Arcade only or midi clock or ext gate, 1 Internal clock
unsigned int seqTempo = 120;
//bool seqOn = true;  //SEQ I ALWAYS ON M8
byte seqLength = 16;
byte writeOctSelect = 3;
byte noteSelect = 0;
byte prevNoteSelect = 0;
bool refreshWriteNotePing = false;

Line <Q16n16> aInterpolate;
int LEDS[4] = { PB12, PB13, PB14, PB15 };
int BUTTONS[5] = { PB3, PB4, PB5, PB6, PB7 };
int KNOBS[4] = { 0, 1, 2, 3 };
int mozziRaw[4] = { 0, 512, 512, 0 };
int oldMozziRaw[4] = { 0, 0, 0, 0 };
bool buttStates[5] = { false, false, false, false, false };
bool oldButtStates[5] = { false, false, false, false, false };
bool ArcadeState = false;
bool oldArcadeState = false;
bool noteIsOn = false; //keep track of number of playing notes
const byte NOTEON = 0x09;
const byte NOTEOFF = 0x08;
const byte MCLOCKTICK = 0x03;

//int oldinternalClockSelect = 100;
bool syncPulse = false;
//int jitterfreq = 0;
byte pageState = 0;
float noteFreq = 0; //value to store current root freq of note
char lfoOutput = 0; //value to store current offset from root
//float lfoOutput = 0; //value to store current offset from root
float aSinFreq = 440; //value to store current final oscillator frequency
int mod_freq = 440;
int mod_ratio = 3;
long fm_intensity = 0;
byte waveformselect = 0;
byte lastNote = 0;
//float modToFMIntensity = 0;
//float FMENVval = 0; //NOT USED
float modDepth = 0;
float freeq = 0;
bool offsetOn = false;
byte lfoDest = 0;
byte lpfCutoff = 100;
byte lpfCutoffMod = 0;
float PitchOffset = 0;
byte LFOWaveSelect = 0;
unsigned long rndTimer = 0;
long int rndFreq = 0;
Q16n16 HDlfoOutputBuffer = 0; //this is a big type for slew manipulation
byte arcadeNote = 0;
bool knobLock[4] = { true, true, true, true };
int lockAnchor[4] = { -99, -99, -99, -99 };
int lockThresh = 50;



//#define ARCADEBUTTON 16
#define ARCADEBUTTON PA15
#define LED PB11 // shows if MIDI is being recieved, is also available as a gate output
#define minusButton 1
#define plusButton 3
#define FMknob  0
#define h4xxKnob  1

#define attackKnob 2
#define releaseKnob 3
#define BUTTON1 0
#define BUTTON2 2
#define BUTTON3 4
#define deadzone 100 //dead zone / notch for bipolar values (on haxx)

//const int MIN_INTENSITY = 700;
//const int MAX_INTENSITY = 10;

//MIDI_CREATE_DEFAULT_INSTANCE();
MIDI_CREATE_INSTANCE(HardwareSerial, Serial1, MIDI);

// use #define for CONTROL_RATE, not a constant

#define CONTROL_RATE 512// powers of 2 please
//#define AUDIO_RATE 32768

// audio sinewave oscillator
Oscil <SIN1024_NUM_CELLS, AUDIO_RATE> aSin(SIN1024_DATA);   //declare Oscillator
Oscil <SIN1024_NUM_CELLS, AUDIO_RATE> aMod(SIN1024_DATA);	//declare Modulator
Oscil <SIN512_NUM_CELLS, CONTROL_RATE> LFO(SIN512_DATA);    //LFO//


// envelope generator
ADSR <CONTROL_RATE, AUDIO_RATE> envelope;					//declare AMP env //
LowPassFilter lpf;
void setUpMidi() {
	MIDI.setHandleNoteOn(HandleDINNoteOn);  // This is where we'll handle Hardware MIDI noteons (Put only the name of the function)
	MIDI.setHandleNoteOff(HandleDINNoteOff);  // This is where well handle hardware midi noteoffs
	MIDI.setHandleStart(handleMIDIClockStart);
	MIDI.setHandleStop(handleMIDIClockStop);
	MIDI.setHandleClock(handleMIDIClock);
	MIDI.begin(MIDI_CHANNEL_OMNI); // Initiate MIDI communications, listen to all channels
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool somethingToPrint = false;
char* stringToPrint = "default string";
int valToPrint = 0;
class gateMidi : public USBMidi {
	void handleNoteOff(unsigned int channel, unsigned int note, unsigned int velocity) {
		HandleNoteOff(note, velocity);
	}
	void handleNoteOn(unsigned int channel, unsigned int note, unsigned int velocity) {
		HandleNoteOn(note, velocity);
	}

	void handleSync() {
		//do a sync
	}



};

gateMidi umidi;



void setup() {

	umidi.registerComponent();
	CompositeSerial.registerComponent();
	USBComposite.begin();


	setUpMidi();

	//pin setup
	pinMode(debugTogglePin, INPUT_PULLUP);
	pinMode(PA0, INPUT_ANALOG);
	pinMode(PA1, INPUT_ANALOG);
	pinMode(PA2, INPUT_ANALOG);
	pinMode(PA3, INPUT_ANALOG);
	for (int i = 0; i < 5; i++) {
		pinMode(BUTTONS[i], INPUT_PULLUP);
	}
	pinMode(ARCADEBUTTON, INPUT_PULLUP);
	for (int i = 0; i < 4; i++) {
		pinMode(LEDS[i], OUTPUT);
	}
	pinMode(LED, OUTPUT);
	if (EEPROM.read(100) == 123) {
		readSeqFromEeprom();
	}

	//mozzi setup
	envelope.setADLevels(200, 240);			// attacks and decays need to be tweaked !!!!!!!!!!!!!!!!!!!!!!!!!!!
	envelope.setTimes(10, 200, 65000, 10); // 65000 is so the note will sustain 65 seconds unless a noteOff comes (it's an unsigned int so it will will overflow at 65535)
	aSin.setFreq(440); // default frequency
	startMozzi(CONTROL_RATE);
	lpf.setResonance(200);
	lpf.setCutoffFreq(100);
	LFO.setFreq(1);
	digitalWrite(LEDS[pageState], HIGH); //show us what pagestate we are in
}

//debugVars
int outPuTTY = 0;
int count = 0;
int globalVal = 0;



void updateControl() {
	//debugToggle = digitalRead(debugTogglePin);
	//if (debugToggle) {
	  //debug();
	//}

	umidi.poll();

	MIDI.read(); //check for DIN midi

	getKnobStates();

	getButtStates();

	handlePageButts();

	handleArcadeButt();

	handleKnob1();

	handleKnob2();

	handleKnob3();

	handleKnob4();

	handleInternalCV();

	if (pageState == 3) {
		//	seqCheckButts(); //only if in seq mode
	}

	// handleSequencer();
}


int updateAudio() {
	long modulation = fm_intensity * aMod.next();
	int output = (envelope.next() * aSin.phMod(modulation)) >> 6;//9 is safe

	output = lpf.next(output);       //TRY PUTTING THIS INLINE WITH THE REST! THEN TRY LIMITING IT WITH AN IF STATEMENT TO AVOID NASTY CLIPPING ARTEFACTS ??
	return (int64_t)output;
}


void loop() {
	audioHook(); // required here


	if (somethingToPrint) {
		debugPort.print(stringToPrint);
		debugPort.println(valToPrint);
		somethingToPrint = false;
	}
}
