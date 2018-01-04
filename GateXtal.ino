

/*
 Name:		GateXtal.ino
 Created:	10/16/2017 9:18:53 AM
 Author:	Daniel Lacey

 TODO
 SEQ needs to trigger first note when turned on
 

 */

//LIBRARIES
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
#include <tables/cos2048_int8.h> // sine table for oscillator
#include <tables/sin2048_int8.h> // cosine table for oscillator
#include <tables/saw2048_int8.h> // saw table for oscillator
#include <tables/triangle2048_int8.h> // triangle table for oscillator
#include <tables/square_analogue512_int8.h> // square table for oscillator
#include <tables/sin512_int8.h> //lofi sine for LFO
#include <tables/brownnoise8192_int8.h> // recorded audio wavetable


byte midiClockDivider = 1;
byte seqClockType = 0; //0 = internal, 1 = gate in / arcadeStep, 2 = midiClock 
unsigned int tempo = 120;
bool seqOn = false;
byte seqLength = 16;

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
int notecounter = 0; //keep track of number of playing notes
const byte NOTEON = 0x09;
const byte NOTEOFF = 0x08;
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

//#define CONTROL_RATE 256// powers of 2 please

// audio sinewave oscillator
Oscil <COS2048_NUM_CELLS, AUDIO_RATE> aSin(COS2048_DATA);   //declare Oscillator
Oscil <COS2048_NUM_CELLS, AUDIO_RATE> aMod(COS2048_DATA);	//declare Modulator
Oscil <SIN512_NUM_CELLS, CONTROL_RATE> LFO(SIN512_DATA);    //LFO

//AutoMap kMapIntensity(0, 1023, MIN_INTENSITY, MAX_INTENSITY);

// envelope generator
ADSR <CONTROL_RATE, AUDIO_RATE> envelope;					//declare AMP env
//ADSR <CONTROL_RATE, AUDIO_RATE> MODenvelope;				//declare FM / MOD env

LowPassFilter lpf;



void setup() {
	for (int i = 0; i < 5; i++) {
		pinMode(BUTTONS[i], INPUT_PULLUP);
	}
	pinMode(ARCADEBUTTON, INPUT_PULLUP);
	for (int i = 0; i < 5; i++) {
		pinMode(LEDS[i], OUTPUT);
	}


	randSeed(); // fresh random
	//pinMode(LED, OUTPUT);
	MIDI.setHandleNoteOn(HandleNoteOn);  // This is where we'll handle Hardware MIDI noteons (Put only the name of the function) 
	MIDI.setHandleNoteOff(HandleNoteOff);  // This is where well handle hardware midi noteoffs
	MIDI.begin(MIDI_CHANNEL_OMNI); // Initiate MIDI communications, listen to all channels

	envelope.setADLevels(255, 240);			// attacks and decays need to be tweaked !!!!!!!!!!!!!!!!!!!!!!!!!!!
	envelope.setTimes(100, 200, 65000, 200); // 65000 is so the note will sustain 65 seconds unless a noteOff comes (it's an unsigned int so it will will overflow at 65535)

	//MODenvelope.setADLevels(255, 127);		//these also need to be tweaked
	//MODenvelope.setTimes(100, 200, 100000, 200); // 10000 is so the note will sustain 10 seconds unless a noteOff comes
	aSin.setFreq(440); // default frequency
	startMozzi(CONTROL_RATE);
	//Serial.begin(9600);

	


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
		//Serial.println("KNOBS LOCKED");
	}
}

void updateControl() {
	usbmidiprocessing(); //check for USB midi notes
	MIDI.read(); //check for DIN midi notes	

	for (int i = 0; i < 4; i++) {
		mozziRaw[i] = mozziAnalogRead(KNOBS[i]); //get knobstates
		if (knobLock[i]) {                       //if this knob is locked
			if (mozziRaw[i] > lockAnchor[i] + lockThresh || mozziRaw[i] < lockAnchor[i] - lockThresh) { // if this knob has wandered far enough from the lock anchor, we can start listening to it
				knobLock[i] = false;

				//Serial.print("Knob nr.");
				//Serial.print(i);
				//Serial.println(" was unlocked");

			}
		}
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
		lockKnobs();
		pageState--;
		digitalWrite(LEDS[(pageState + 1) % 4], LOW);
		pageState = pageState % 4;
		//Serial.print("page ");
		//Serial.println(pageState);
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
		//Serial.print("page ");
		//Serial.println(pageState);
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
		arcadeNote = rand(20, 80);
		if (seqOn && seqClockType == 1) {

		}
		HandleNoteOn(1, arcadeNote, 127);
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
				rndFreq = (val*-1 +1024)<<5; //invert and scale down

				freeq = val;
				LFO.setFreq(freeq / 50);
				//CONTROL LFO RATE
				//modToFMIntensity = val;
				//modToFMIntensity = modToFMIntensity / 1000;
				//Serial.println(modToFMIntensity);
				break;
			case 2:
				envelope.setAttackTime(val + 18);
				//Serial.print("ATK ");
				//Serial.println(val+18);
				break;
			case 3:
				//MODenvelope.setAttackTime(val+10);
				//Serial.print("MODATK ");
				//Serial.println(val+10);
				break;

			}
		

		oldMozziRaw[FMknob] = mozziRaw[FMknob];
	}


	////////////////////////
	//HANDLE HAXX KNOB    // 2
	////////////////////////
	//Serial.println(buttStates[BUTTON1]);
	if (mozziRaw[h4xxKnob] != oldMozziRaw[h4xxKnob] && !knobLock[h4xxKnob]) {       //if h4xxknob was moved
		int val = mozziRaw[h4xxKnob];

		if (buttStates[BUTTON1]) {
			//Serial.print("RES ");
			lpf.setResonance(val >> 2);
			//waveformselect = mozziRaw[h4xxKnob] >> 8; // 0 - 1024 to 0 - 4

				//setWaveForm(waveformselect);
		}
		else {
			switch (pageState) {
			case 0:
				lpfCutoff = val >> 2;
				lpf.setCutoffFreq(lpfCutoff);
				//Serial.println(val >> 2);
				//Serial.println(val >> 2);
				break;
			case 1:
				//CONTROL LFO DEPTH BIPOLAR M8
				freeq = val;
				modDepth = freeq / 1000;
				offsetOn = modDepth;
				//Serial.println(offsetOn);
				break;
			case 2:
				envelope.setDecayTime(mozziRaw[h4xxKnob]);
				break;
			case 3:
				//MODenvelope.setDecayTime(mozziRaw[h4xxKnob]);
				break;
			}
		}

		oldMozziRaw[h4xxKnob] = mozziRaw[h4xxKnob];
	}

	//////////////////////
	//HANDLE ATTACK KNOB// 3
	//////////////////////
	if (mozziRaw[attackKnob] != oldMozziRaw[attackKnob] && !knobLock[attackKnob]) { //if there was a change to attackKnob
		int val = mozziRaw[attackKnob];
		if (buttStates[BUTTON2]) {
			jitterfreq = val;
			PitchOffset = (float(jitterfreq) - 512);
			if (PitchOffset > -deadzone && PitchOffset < deadzone) {		//if we are in the deadzone
				PitchOffset = 0;											//set value to zero
			}
			else if (PitchOffset <= -deadzone) {      //if its below lower deadzone
				PitchOffset = PitchOffset + deadzone;  //add deadzone to it so it starts from 0
			}
			else {									//if none of above cases apply, it means we are above deadzone
				PitchOffset = PitchOffset - deadzone;  //subtract deadzone from it so it starts from 0
			}
		}
		else {
			switch (pageState) {           //knob does different things depending on pagestate
			case 0:
				mod_ratio = val >> 7;

				//Serial.println(PitchOffset);
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
				//	MODenvelope.setSustainLevel(val);
				//	MODenvelope.setDecayLevel(val);
				break;

			default:
				break;
			}
		}
		oldMozziRaw[attackKnob] = mozziRaw[attackKnob];
	}
	///////////////////////
	//HANDLE RELEASE KNOB//  4
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
			//Serial.print("REL ");
			//Serial.println(mozziRaw[releaseKnob]);
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
			break;

		default:
			break;
		}
		oldMozziRaw[releaseKnob] = mozziRaw[releaseKnob];
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
	//Serial.println(LFO.next());

	//lfoOutput = lfoOutput;// *modDepth;
	//MODenvelope.update(); //WIP
	//Serial.println(modDepth);
	if (offsetOn) {
		if (LFOWaveSelect == 0) {
			lfoOutput = LFO.next();
		}
		else {
			//RANDOM
			
			if (mozziMicros() - rndTimer > (rndFreq<<3) ) {
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
			//Serial.println(rndFreq);
			
		}
		
		if (lfoDest == 0) {
			
			aSinFreq = noteFreq + (lfoOutput * modDepth);
			aSin.setFreq(aSinFreq);
		}
		else {
			aSinFreq = noteFreq;
		}

		if (lfoDest == 1){
			

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
//Serial.print("aModFREQ = ");
//Serial.println(aSinFreq);

//lpf.setCutoffFreq(MODenvelope.next());

//	Serial.print("MOD: ");
//	Serial.print(MODenvelope.next());
//	Serial.print("  AMP: ");
//	Serial.println(envelope.next());
//Serial.print("FM_INTENSITY = ");
//Serial.print(fm_intensity);
//Serial.print("  JITTERFREQ = ");
//Serial.println(jitterfreq);
if (seqOn) {
	handleSequencer();
}

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