/*
 Name:		GateXtal.ino
 Created:	10/16/2017 9:18:53 AM
 Author:	Daniel Lacey
*/

//LIBRARIES
#include <MIDI.h>
#include <MozziGuts.h>
#include <Oscil.h> // oscillator template
#include <mozzi_midi.h>
#include <ADSR.h>
#include <AutoMap.h> // maps unpredictable inputs to a range
#include <LowPassFilter.h>

//TABLES
#include <tables/cos2048_int8.h> // sine table for oscillator
#include <tables/sin2048_int8.h> // cosine table for oscillator
#include <tables/saw2048_int8.h> // saw table for oscillator
#include <tables/triangle2048_int8.h> // triangle table for oscillator
#include <tables/square_analogue512_int8.h> // square table for oscillator
#include <tables/sin512_int8.h> //lofi sine for LFO

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
int notecounter = 0; //keep track of number of playing notes
const byte NOTEON = 0x09;
const byte NOTEOFF = 0x08;
int jitterfreq = 0;
byte pageState = 0;
float noteFreq = 0; //value to store current root freq of note
float offsetFreq = 0; //value to store current offset from root
float aSinFreq = 440; //value to store current final oscillator frequency
int mod_ratio = 3;
long fm_intensity = 0;
byte waveformselect = 0;

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
Oscil <COS2048_NUM_CELLS, AUDIO_RATE> aSin(COS2048_DATA);
Oscil <COS2048_NUM_CELLS, AUDIO_RATE> aMod(COS2048_DATA);

AutoMap kMapIntensity(0, 1023, MIN_INTENSITY, MAX_INTENSITY);

// envelope generator
ADSR <CONTROL_RATE, AUDIO_RATE> envelope;
ADSR <CONTROL_RATE, AUDIO_RATE> FMenvelope;

LowPassFilter lpf;



void setup() {
	pinMode(LED, OUTPUT);
	MIDI.setHandleNoteOn(HandleNoteOn);  // This is where we'll handle Hardware MIDI noteons (Put only the name of the function) 
	MIDI.setHandleNoteOff(HandleNoteOff);  // This is where well handle hardware midi noteoffs
	MIDI.begin(MIDI_CHANNEL_OMNI); // Initiate MIDI communications, listen to all channels
	envelope.setADLevels(255, 240);			// attacks and decays need to be tweaked !!!!!!!!!!!!!!!!!!!!!!!!!!!
	envelope.setTimes(100, 200, 100000, 200); // 10000 is so the note will sustain 10 seconds unless a noteOff comes
	FMenvelope.setADLevels(255, 127);		//these also need to be tweaked
	FMenvelope.setTimes(100, 200, 10000, 200); // 10000 is so the note will sustain 10 seconds unless a noteOff comes
	aSin.setFreq(440); // default frequency
	startMozzi(CONTROL_RATE);
	Serial.begin(9600);

	for (int i = 0; i < 5; i++) {
		pinMode(BUTTONS[i], INPUT_PULLUP);
	}

	pinMode(ARCADEBUTTON, INPUT_PULLUP);
	for (int i = 0; i < 5; i++) {
		pinMode(LEDS[i], OUTPUT);
	}


	pinMode(LED_BUILTIN_TX, INPUT); // Deactivate RX LED
	pinMode(LED_BUILTIN_RX, INPUT); // Deactivate RX LED

	digitalWrite(LEDS[pageState], HIGH); //show us what pagestate we are in
	lpf.setResonance(200);
	lpf.setCutoffFreq(100);

}

void setWaveForm(byte waveNumber) {
	switch (waveNumber) {
	case 0:
		aSin.setTable(SIN2048_DATA);
		aMod.setTable(SIN2048_DATA);
		Serial.print("waveform = SINE");

		break;
	case 1:
		aSin.setTable(TRIANGLE2048_DATA);
		aMod.setTable(TRIANGLE2048_DATA);

		Serial.print("waveform = TRIANGLE");

		break;
	case 2:
		aSin.setTable(SAW2048_DATA);
		Serial.print("waveform = SAW");
		aMod.setTable(SAW2048_DATA);
		break;
	case 3:
		aSin.setTable(SQUARE_ANALOGUE512_DATA);
		Serial.print("waveform = SQUARE");
		aMod.setTable(SQUARE_ANALOGUE512_DATA);
		break;

	}
}

void updateControl() {
	usbmidiprocessing(); //check for USB midi notes
	MIDI.read(); //check for DIN midi notes	

	for (int i = 0; i < 4; i++) {
		mozziRaw[i] = mozziAnalogRead(KNOBS[i]); //get knobstates
	}

	for (int i = 0; i < 5; i++) {
		buttStates[i] = !digitalRead(BUTTONS[i]); //get buttstates
		if (buttStates[i]) {
			//	Serial.println(i);
		}

		ArcadeState = !digitalRead(ARCADEBUTTON);

	}
	//HANDLE BUTTONS FOR CHANGING PAGE
	if (buttStates[minusButton] && !oldButtStates[minusButton]) { //if page- button pressed and it wasn't previously pressed
		pageState--;
		digitalWrite(LEDS[(pageState + 1) % 4], LOW);
		pageState = pageState % 4;
		Serial.print("page ");
		Serial.println(pageState);
		digitalWrite(LEDS[pageState], HIGH);
		oldButtStates[minusButton] = buttStates[minusButton];	//remember this happened
	}
	else if (!buttStates[minusButton] && oldButtStates[minusButton]) { //if page- button was released
		oldButtStates[minusButton] = buttStates[minusButton];	//remember this happened
	}

	if (buttStates[plusButton] && !oldButtStates[plusButton]) { //if page- button pressed and it wasn't previously pressed
		pageState++;
		digitalWrite(LEDS[(pageState - 1)], LOW);
		pageState = pageState % 4;
		Serial.print("page ");
		Serial.println(pageState);
		digitalWrite(LEDS[pageState], HIGH);
		oldButtStates[plusButton] = buttStates[plusButton];	//remember this happened
	}
	else if (!buttStates[plusButton] && oldButtStates[plusButton]) {
		oldButtStates[plusButton] = buttStates[plusButton];	//remember this happened
	}
	//////////////////////
	//HANDLE FM KNOB (0)//
	//////////////////////



	if (mozziRaw[FMknob] != oldMozziRaw[FMknob]) {
		int val = mozziRaw[FMknob];

		if (ArcadeState) {								//if Arcedebutton is Pressed
		}
		else {
			switch (pageState) {
			case 0:
				fm_intensity = (float(mozziRaw[FMknob]));// *FMenvelope.next();
				break;
			case 1:
				//CONTROL LFO RATE
				break;
			case 2:
				envelope.setAttackTime(mozziRaw[FMknob]);
				break;
			case 3:
				FMenvelope.setAttackTime(mozziRaw[FMknob]);
				break;

			}
		}

		oldMozziRaw[FMknob] = mozziRaw[FMknob];
	}


	////////////////////////
	//HANDLE HAXX KNOB (1)//
	////////////////////////
	//Serial.println(buttStates[BUTTON1]);
	if (mozziRaw[h4xxKnob] != oldMozziRaw[h4xxKnob]) {       //if h4xxknob was moved
		int val = mozziRaw[h4xxKnob];
		
		if (buttStates[BUTTON1]) {
			Serial.println("ACTIVATE");
		//waveformselect = mozziRaw[h4xxKnob] >> 8; // 0 - 1024 to 0 - 4

			//setWaveForm(waveformselect);
		}
		else {
			switch (pageState) {
			case 0:
				lpf.setCutoffFreq(val>>2);
				break;
			case 1:
				//CONTROL LFO DEPTH BIPOLAR M8
				break;
			case 2:
				envelope.setDecayTime(mozziRaw[h4xxKnob]);
				break;
			case 3:
				FMenvelope.setDecayTime(mozziRaw[h4xxKnob]);
				break;
			}
		}

		oldMozziRaw[h4xxKnob] = mozziRaw[h4xxKnob];
	}

	//////////////////////
	//HANDLE ATTACK KNOB//
	//////////////////////
	if (mozziRaw[attackKnob] != oldMozziRaw[attackKnob]) { //if there was a change to attackKnob
		int val = mozziRaw[attackKnob];
		switch (pageState) {           //knob does different things depending on pagestate
		case 0: // THIS KNOB IS MOD FREQ OFFSET
			jitterfreq = val;
			offsetFreq = (float(jitterfreq) - 512);
			if (offsetFreq > -deadzone && offsetFreq < deadzone) {		//if we are in the deadzone
				offsetFreq = 0;											//set value to zero
			}
			else if (offsetFreq <= -deadzone) {      //if its below lower deadzone
				offsetFreq = offsetFreq + deadzone;  //add deadzone to it so it starts from 0
			}
			else {									//if none of above cases apply, it means we are above deadzone
				offsetFreq = offsetFreq - deadzone;  //subtract deadzone from it so it starts from 0
			}

			//Serial.println(offsetFreq);
			break;

		case 1:
			//CONTROL LFO DEPTH
			break;
		case 2:
			envelope.setSustainLevel(mozziRaw[attackKnob]);
			break;
		case 3:
			FMenvelope.setSustainLevel(mozziRaw[attackKnob]);
			break;

		default:
			break;
		}
		oldMozziRaw[attackKnob] = mozziRaw[attackKnob];
	}
	///////////////////////
	//HANDLE RELEASE KNOB//
	///////////////////////
	if (mozziRaw[releaseKnob] != oldMozziRaw[releaseKnob]) { //if there was a change to releaseKnob
		int val = mozziRaw[releaseKnob];
		switch (pageState) {           //knob does different things depending on pagestate

		case 0:
			//envelope.setReleaseTime(mozziRaw[releaseKnob]);
			waveformselect = val >> 8; // 0 - 1024 to 0 - 4
			setWaveForm(waveformselect);
			//Serial.print("REL ");
			//Serial.println(mozziRaw[releaseKnob]);
			break;

		case 1:
			//CONTROL LFO TARGET
			break;
		case 2:
			envelope.setReleaseTime(mozziRaw[releaseKnob]);
			break;
		case 3:
			break;

		default:
			break;
		}
		oldMozziRaw[releaseKnob] = mozziRaw[releaseKnob];
	}

	envelope.update();
	//FMenvelope.update(); //WIP
	aSinFreq = noteFreq + offsetFreq;
	int mod_freq = aSinFreq * mod_ratio;
	aMod.setFreq(mod_freq);
	//Serial.print("env: ");
	//Serial.println(envelope.next());
}


int updateAudio() {
	long modulation = fm_intensity * aMod.next();
	//  return aCarrier.phMod(modulation); // phMod does the FM

	char output = (envelope.next() * aSin.phMod(modulation)) >> 9;
	output = lpf.next(output);       //TRY PUTTING THIS INLINE WITH THE REST! THEN TRY LIMITING IT WITH AN IF STATEMENT TO AVOID NASTY CLIPPING ARTEFACTS
	return (int)output;

	//return (int)(envelope.next() * aSin.phMod(modulation)) >> 9;
	//  return (int) (envelope.next() * aSin.next())>>8;
}


void loop() {
	audioHook(); // required here
}