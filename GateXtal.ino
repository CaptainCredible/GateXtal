//#define AUDIO_RATE 1000

/*
 Name:		GateXtal.ino
 Created:	10/16/2017 9:18:53 AM
 Author:	Daniel Lacey

 TODO
 SEQ needs to trigger first note when turned on
 */

 //LIBRARIES
#include <EEPROM\EEPROM.h>
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

byte noteToWrite = 0;
byte octOffset = 0;
byte sequence[64] = { 40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51,40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51 };
unsigned int seqIncrement = 0; //counter to keep track of when next step should come
byte seqCurrentStep = 0;
byte midiClockTicks = 0;
byte midiSeqNoteLength = 23;
byte seqNoteLength = 60;
byte midiClockDivider = 1;
byte seqClockType = 0; //0 = Arcade only or midi clock or ext gate, 1 Internal clock
unsigned int seqTempo = 120;
//bool seqOn = true;  //SEQ I ALWAYS ON M8
byte seqLength = 16;
byte writeOctSelect = 3;
byte noteSelect = 0;
byte prevNoteSelect = 0;
bool refreshWriteNotePing = false;

Line <Q16n16> aInterpolate;
int LEDS[4] = { 14,15,2,3 };
//int BUTTONS[5] = { 8,7,6,5,4 };
int BUTTONS[5] = { 4,5,6,7,8 };
int KNOBS[4] = { 0,1,2,3 };
int mozziRaw[4] = { 0,0,0,0 };
int oldMozziRaw[4] = { 0,0,0,0 };
bool buttStates[5] = { false, false, false, false, false };
bool oldButtStates[5] = { false, false, false, false, false };
bool ArcadeState = false;
bool oldArcadeState = false;
bool noteIsOn = false; //keep track of number of playing notes
const byte NOTEON = 0x09;
const byte NOTEOFF = 0x08;
const byte MCLOCKTICK = 0x03;

int jitterfreq = 0;
byte pageState = 0;
float noteFreq = 0; //value to store current root freq of note
char lfoOutput = 0; //value to store current offset from root
//float lfoOutput = 0; //value to store current offset from root
float aSinFreq = 440; //value to store current final oscillator frequency
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
int lockAnchor[4] = { -99,-99,-99,-99 };
int lockThresh = 50;


#define ARCADEBUTTON 16
#define LED 10 // shows if MIDI is being recieved, is also available as a gate output
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

const int MIN_INTENSITY = 700;
const int MAX_INTENSITY = 10;

MIDI_CREATE_DEFAULT_INSTANCE();

// use #define for CONTROL_RATE, not a constant

#define CONTROL_RATE 128// powers of 2 please


// audio sinewave oscillator
Oscil <SIN1024_NUM_CELLS, AUDIO_RATE> aSin(SIN1024_DATA);   //declare Oscillator
Oscil <SIN1024_NUM_CELLS, AUDIO_RATE> aMod(SIN1024_DATA);	//declare Modulator
Oscil <SIN512_NUM_CELLS, CONTROL_RATE> LFO(SIN512_DATA);    //LFO


// envelope generator
ADSR <CONTROL_RATE, AUDIO_RATE> envelope;					//declare AMP env
LowPassFilter lpf;



void setup() {
	int magicNumber = EEPROM.read(100);
	if (magicNumber == 123) {
		readSeqFromEeprom();
	}
	for (int i = 0; i < 5; i++) {
		pinMode(BUTTONS[i], INPUT_PULLUP);
	}
	pinMode(ARCADEBUTTON, INPUT_PULLUP);
	for (int i = 0; i < 5; i++) {
		pinMode(LEDS[i], OUTPUT);
	}


	//randSeed(); // fresh random

	MIDI.setHandleNoteOn(HandleDINNoteOn);  // This is where we'll handle Hardware MIDI noteons (Put only the name of the function) 
	MIDI.setHandleNoteOff(HandleDINNoteOff);  // This is where well handle hardware midi noteoffs
	MIDI.setHandleClock(HandleMIDIClock);
	MIDI.begin(MIDI_CHANNEL_OMNI); // Initiate MIDI communications, listen to all channels

	envelope.setADLevels(255, 240);			// attacks and decays need to be tweaked !!!!!!!!!!!!!!!!!!!!!!!!!!!
	envelope.setTimes(100, 200, 65000, 200); // 65000 is so the note will sustain 65 seconds unless a noteOff comes (it's an unsigned int so it will will overflow at 65535)

	aSin.setFreq(440); // default frequency
	startMozzi(CONTROL_RATE);




	pinMode(LED_BUILTIN_TX, INPUT); // Deactivate RX LED
	pinMode(LED_BUILTIN_RX, INPUT); // Deactivate RX LED

	digitalWrite(LEDS[pageState], HIGH); //show us what pagestate we are in
	lpf.setResonance(200);
	lpf.setCutoffFreq(100);
	LFO.setFreq(1);
}

void lockKnobs() {
	for (int i = 0; i < 4; i++) {
		lockAnchor[i] = mozziRaw[i];
		knobLock[i] = true;
		
	}
}


#define TICK 15
#define STOP 11

void updateControl() {

	usbmidiprocessing(); //check for USB midi
	MIDI.read(); //check for DIN midi

	for (int i = 0; i < 4; i++) {
		mozziRaw[i] = mozziAnalogRead(KNOBS[i]); //get knobstates
		if (knobLock[i]) {                       //if this knob is locked
			if (mozziRaw[i] > lockAnchor[i] + lockThresh || mozziRaw[i] < lockAnchor[i] - lockThresh) { // if this knob has wandered far enough from the lock anchor, we can start listening to it
				knobLock[i] = false;
			}
		}
	}

	for (int i = 0; i < 5; i++) {
		buttStates[i] = !digitalRead(BUTTONS[i]); //get buttstates
		if (buttStates[i]) {
		}

		ArcadeState = !digitalRead(ARCADEBUTTON);

	}
	//HANDLE BUTTONS FOR CHANGING PAGE
	if (buttStates[minusButton] && !oldButtStates[minusButton]) { //if page- button pressed and it wasn't previously pressed
		lockKnobs();
		pageState--;
		digitalWrite(LEDS[(pageState + 1) % 4], LOW);
		pageState = pageState % 4;
		digitalWrite(LEDS[pageState], HIGH);
		oldButtStates[minusButton] = buttStates[minusButton];	//remember this happened
	}
	else if (!buttStates[minusButton] && oldButtStates[minusButton]) { //if page- button was released
		oldButtStates[minusButton] = buttStates[minusButton];	//remember this happened
	}

	if (buttStates[plusButton] && !oldButtStates[plusButton]) { //if page- button pressed and it wasn't previously pressed
		lockKnobs();
		pageState++;
		digitalWrite(LEDS[(pageState - 1)], LOW);
		pageState = pageState % 4;
		digitalWrite(LEDS[pageState], HIGH);
		oldButtStates[plusButton] = buttStates[plusButton];	//remember this happened
	}
	else if (!buttStates[plusButton] && oldButtStates[plusButton]) {
		oldButtStates[plusButton] = buttStates[plusButton];	//remember this happened
	}

	///////////////////////////
	/// HANDLE ARCADE BUTTON //
	///////////////////////////

	if (ArcadeState && !oldArcadeState) {								//if Arcedebutton is Pressed
		
		if (pageState == 3 && buttStates[BUTTON2]) {
			writeToSeq();
		} else {
		playNextStep();
		}

		oldArcadeState = ArcadeState;
	}
	else if (!ArcadeState && oldArcadeState) {
		HandleNoteOff(1, arcadeNote, 0);
		oldArcadeState = ArcadeState;
	}

	//////////////////////
	//HANDLE FM KNOB    //  1
	//////////////////////



	if (mozziRaw[FMknob] != oldMozziRaw[FMknob] && !knobLock[FMknob]) {
		int val = mozziRaw[FMknob];



		switch (pageState) {
		case 0:
			fm_intensity = (float(val));// *FMenvelope.next();
			break;
		case 1:
			rndFreq = (val*-1 + 1024) << 5; //invert and scale down

			freeq = val;
			LFO.setFreq(freeq / 50);
			//CONTROL LFO RATE
			//modToFMIntensity = val;
			//modToFMIntensity = modToFMIntensity / 1000;
			break;
		case 2:
			envelope.setAttackTime(val + 18);
			break;
		case 3:
			//LENGTH PERHAPS
			//MODenvelope.setAttackTime(val+10);
			break;

		}


		oldMozziRaw[FMknob] = mozziRaw[FMknob];
	}


	////////////////////////
	//HANDLE HAXX KNOB    // 2
	////////////////////////

	if (mozziRaw[h4xxKnob] != oldMozziRaw[h4xxKnob] && !knobLock[h4xxKnob]) {       //if h4xxknob was moved
		int val = mozziRaw[h4xxKnob];



		switch (pageState) {
		case 0:
			lpfCutoff = val >> 2;
			lpf.setCutoffFreq(lpfCutoff);
			if (buttStates[BUTTON1]) {
				lpf.setResonance(val >> 2);
			}
			break;
		case 1:
			//CONTROL LFO DEPTH BIPOLAR M8
			freeq = val;
			modDepth = freeq / 1000;
			offsetOn = modDepth;
			break;
		case 2:
			//envelope.setDecayTime(mozziRaw[h4xxKnob]);
			envelope.setDecayTime(val);
			break;
		case 3:
			if (buttStates[BUTTON1]) {
				if (seqClockType != 2) {//val = 1024 - val; //invert val
					//int divisor = val >> 1;
					//seqNoteLength = seqTempo / divisor; //make notelength a division of val 
					seqNoteLength = map(val, 0, 1024, 0, seqTempo);
					//midiSeqNoteLength = map(val, 0, 1024, 0, 24);
					midiSeqNoteLength = val >> 5; // scale val 0-32
				}
				else {
					midiSeqNoteLength = map(val, 0, 1023, 0, 24);
					//midiSeqNoteLength = val >> 5; // scale val 0-32
				}
			}
			//determin clock mode and tempo
			if (!buttStates[BUTTON1]) {
				if (val < 4) {
					seqClockType = 0;
				}
				else {
					seqClockType = 1;				 //INTERNAL
					seqTempo = (val >> 3) + 3;       //SCALE DOWN
					seqTempo = (seqTempo*-1) + 130;	 //INVERT
				}
			}
			break;
		}
		oldMozziRaw[h4xxKnob] = mozziRaw[h4xxKnob];
	}

	//////////////////////
	//HANDLE ATTACK KNOB// 3
	//////////////////////
	if (mozziRaw[attackKnob] != oldMozziRaw[attackKnob] && !knobLock[attackKnob]) { //if there was a change to attackKnob
		int val = mozziRaw[attackKnob];
		
			switch (pageState) {           //knob does different things depending on pagestate
			case 0:
				mod_ratio = (val >> 7) + 1;

				break;

			case 1:
				//CONTROL LFO DEST
				lfoDest = val >> 8;   //4 different Destinations
				break;
			case 2:
				envelope.setSustainLevel(val);
				envelope.setDecayLevel(val);
				break;
			case 3:
				
				if (buttStates[BUTTON2]) {
					setWriteNote(val >> 6);
				}
				else {
					//some other function !!
				}

				//	MODenvelope.setSustainLevel(val);
				//	MODenvelope.setDecayLevel(val);
				break;

			default:
				break;
			
		}
		oldMozziRaw[attackKnob] = mozziRaw[attackKnob];
	}
	///////////////////////
	//HANDLE KNOB    4   //
	///////////////////////
	if (mozziRaw[releaseKnob] != oldMozziRaw[releaseKnob] && !knobLock[releaseKnob]) { //if there was a change to releaseKnob
		int val = mozziRaw[releaseKnob];
		switch (pageState) {           //knob does different things depending on pagestate

		case 0:
			//envelope.setReleaseTime(mozziRaw[releaseKnob]);
			if (val >> 8 != waveformselect) {
				waveformselect = val >> 8; // 0 - 1024 to 0 - 4
				setWaveForm(waveformselect);
			}

			break;

		case 1:
			if (val < 512) {
				LFOWaveSelect = 0;
			}
			else {
				LFOWaveSelect = 1;
			}
			break;
		case 2:
			envelope.setReleaseTime(mozziRaw[releaseKnob]);
			break;
		case 3:
			if (val >> 7 != writeOctSelect) {
				writeOctSelect = val >> 7; //0-8

				refreshWriteNotePing = true;

				setWriteNote(noteSelect);
			}
			break;

		default:
			break;
		}
		oldMozziRaw[releaseKnob] = mozziRaw[releaseKnob];
	}

	//handle seqbutts if in seqmode:
	if (pageState == 3) {
		seqCheckButts();
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

//HANDLE INTERNAL "CV"
	envelope.update();
	
	if (offsetOn) {
		if (LFOWaveSelect == 0) {
			lfoOutput = LFO.next();
		}
		else {
			//RANDOM

			if (mozziMicros() - rndTimer > (rndFreq << 3)) {
				int lfoOutputBUFFER = rand(0, 244); //if depth adjusts this we can skip a float calc later
				//int lfoOutputBUFFER = rand(0, mozziRaw[1]); //if depth adjusts this we can skip a float calc later
				//int slew = ((invert(mozziRaw[3]))>>2)+1; //invert the value and drop it down by two bits and make sure it doesnt go under 1
				HDlfoOutputBuffer = Q16n0_to_Q16n16(lfoOutputBUFFER);
				rndTimer = mozziMicros();

			}
			Q16n16 slew = ((mozziRaw[3] >> 2)*-1) + 256;
			aInterpolate.set(HDlfoOutputBuffer, slew);
			Q16n16 interpolatedLfoOutputBUFFER = aInterpolate.next();
			lfoOutput = Q16n16_to_Q16n0(interpolatedLfoOutputBUFFER) - 128; //scaled back down to int and offset to -128 to 128


		}

		if (lfoDest == 0) {

			aSinFreq = noteFreq + (lfoOutput * modDepth);
			aSin.setFreq(aSinFreq);
		}
		else {
			aSinFreq = noteFreq;
		}

		if (lfoDest == 1) {


			int cutoffBuffer = lpfCutoff + (lfoOutput + 126 * modDepth);
			if (cutoffBuffer < 0) {
				cutoffBuffer = 0;
			}
			else if (cutoffBuffer > 254) {
				cutoffBuffer = 254;
			}
			//lpfCutoff = (lfoOutput + 126)*modDepth;
			lpf.setCutoffFreq(cutoffBuffer);
		}
		if (lfoDest == 2) {

			//fm_intensity = (float(val));// *FMenvelope.next();
			fm_intensity = (float((lfoOutput + 128)*modDepth));
			jitterfreq = 0;



		}
	}

	int mod_freq = aSinFreq * mod_ratio;
	aMod.setFreq(mod_freq);
	handleSequencer();
	
}


int updateAudio() {
	long modulation = fm_intensity * aMod.next();
	//  return aCarrier.phMod(modulation); // phMod does the FM

	char output = (envelope.next() * aSin.phMod(modulation)) >> 9;//9 is safe
	output = lpf.next(output);       //TRY PUTTING THIS INLINE WITH THE REST! THEN TRY LIMITING IT WITH AN IF STATEMENT TO AVOID NASTY CLIPPING ARTEFACTS ??
	return (int)output;

	//return (int)(envelope.next() * aSin.phMod(modulation)) >> 9;
	//  return (int) (envelope.next() * aSin.next())>>8;
}


void loop() {
	audioHook(); // required here
}