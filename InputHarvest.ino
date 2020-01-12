
void getKnobStates() {
	for (int i = 0; i < 4; i++) {
		int currentVal = mozziAnalogRead(KNOBS[i]);
		int currentDiff = currentVal - mozziRaw[i];
		if (abs(currentDiff) >= 50) {
			valToPrint = currentVal>>2;
			mozziRaw[i] = currentVal; //get knobs
		}



		//safeguard against noise!!!


		if (knobLock[i]) {                       //if this knob is lock
			if (mozziRaw[i] > lockAnchor[i] + lockThresh || mozziRaw[i] < lockAnchor[i] - lockThresh) { // if this knob has wandered far enough from the lock anchor, we can start listening to it
				knobLock[i] = false;
			}
		}
	}
}

void getButtStates() {
	for (int i = 0; i < 5; i++) {
		buttStates[i] = !digitalRead(BUTTONS[i]); //get buttstates
		if (buttStates[i]) {
		}
	}

	ArcadeState = !digitalRead(ARCADEBUTTON);
}

void handlePageButts() {

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
}

void handleArcadeButt() {
	if (ArcadeState && !oldArcadeState) {								//if Arcedebutton is Pressed

		if (pageState == 3 && buttStates[BUTTON2]) {
			writeToSeq();
		}
		else if (!internalClockSelect && !midiClockRunning) {
			playNextStep();
		}
		else {
			seqCurrentStep = seqLength - 1; //move to the last step
			seqIncrement = seqTempo;    //prime incrementor to roll over and thus trigger first step
			midiClockTicks = 0;			//prime incrementor to roll over and thus trigger first step
		}

		oldArcadeState = ArcadeState;
	}
	else if (!ArcadeState && oldArcadeState) {
		if (!writeMode) {
			HandleNoteOff(arcadeNote, 0);
		}
		oldArcadeState = ArcadeState;
	}

}

float smoothness = 0.995f;
Smooth <unsigned int> mySmoothKnob1(smoothness);

void handleKnob1() {
	if (mozziRaw[FMknob] != oldMozziRaw[FMknob] && !knobLock[FMknob]) {
		//int val = mySmoothKnob1.next(mozziRaw[FMknob]>>2);
		int val = mozziRaw[FMknob];
		stringToPrint = "FM knob moved ";
		somethingToPrint = true;

		//


		switch (pageState) {
		case 0:
			//fm_intensity = (float(val));// *FMenvelope.next();
			fm_intensity = val>>2;// *FMenvelope.next();

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
			if (internalClockSelect) {
				seqTempo = (val >> 3) + 3;       //SCALE DOWN
				seqTempo = (seqTempo*-1) + 130;	 //INVERT
			}
			else {
				byte divisor = (val >> 7) % 4;	//divisor is 01230123

				int tempVal = val - 512; //scale around 0;
				if (tempVal <= 0) {

					midiClockStepSize = 24 >> divisor;
					//////Serial.print(" four ");
				}
				else {
					//divisor = 3 - divisor;
					midiClockStepSize = 16 >> divisor;
					////Serial.print(" three ");

				}
				////Serial.print("divisor ");
				////Serial.println(divisor);
			}
			break;

		}


		oldMozziRaw[FMknob] = mozziRaw[FMknob];
	}
}

void handleKnob2() {
	////////////////////////
	//HANDLE KNOB 2       // (FILTER)
	////////////////////////

	if (mozziRaw[h4xxKnob] != oldMozziRaw[h4xxKnob] && !knobLock[h4xxKnob]) {       //if h4xxknob was moved
		int val = mozziRaw[h4xxKnob] >> 2;



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
			seqNoteLength = map(val, 0, 1024, 0, seqTempo);
			midiSeqNoteLength = val >> 5; // scale val 0-32


			break;
		}
		oldMozziRaw[h4xxKnob] = mozziRaw[h4xxKnob];
	}
}

void handleKnob3() {
	//////////////////////
	//HANDLE KNOB 3     // (MOD OCT)
	//////////////////////
	if (mozziRaw[attackKnob] != oldMozziRaw[attackKnob] && !knobLock[attackKnob]) { //if there was a change to attackKnob
		int val = mozziRaw[attackKnob] >> 2;
		globalVal = val;
		switch (pageState) {           //knob does different things depending on pagestate
		case 0:
			if (buttStates[BUTTON2]) {
				if (val >> 7 != octTranspose) {
					octTranspose = val >> 7;
					octTranspose = octTranspose - 4;
					int octedNote = constrain(sequence[seqCurrentStep] + (octTranspose * 12), 0, 127);
					if (noteIsOn) {	//if a note is playing
						legato(octedNote); //slide to new note without retrigging ADSR
					}
					else { //if no note is currently on
						   //new octTranspose will be used next step
					}
				}

				else {

				}
			}
			else {
				mod_ratio = (val >> 6);
			}



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
			}

			break;

		default:
			break;

		}
		oldMozziRaw[attackKnob] = mozziRaw[attackKnob];
	}
}

void handleKnob4() {
	///////////////////////
	//HANDLE KNOB    4   // (waveform)
	///////////////////////
	if (mozziRaw[releaseKnob] != oldMozziRaw[releaseKnob] && !knobLock[releaseKnob]) { //if there was a change to releaseKnob
		int val = mozziRaw[releaseKnob] >> 2;
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
				if (buttStates[BUTTON2] && writeMode) {
					writeOctSelect = val >> 7; //0-8
					refreshWriteNotePing = true;
					setWriteNote(noteSelect);
				}
				else if (buttStates[BUTTON2]) {
				}
				else { // if knob is twiddled and no butts are true
					   //Serial.println("no butts are true m8");
					   //legatOct(val);

					octTranspose = val >> 7; //0-8
					writeOctSelect = octTranspose;
					octTranspose = octTranspose - 4;
					int octedNote = constrain(sequence[seqCurrentStep] + (octTranspose * 12), 0, 127);
					if (noteIsOn) {	//if a note is playing
						legato(octedNote); //slide to new note without retrigging ADSR
					}
					else { //if no note is currently on
						   //new octTranspose will be used next step
					}
				}
			}
			break;

		default:
			break;
		}
		oldMozziRaw[releaseKnob] = mozziRaw[releaseKnob];
	}

}

void handleInternalCV() {
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
			//jitterfreq = 0; 



		}
	}
	if (mod_ratio < 8) {
		byte temp = 8 - mod_ratio;
		mod_freq = aSinFreq;
		mod_freq = mod_freq >> temp;

	}
	else {
		byte temp = mod_ratio - 7;
		mod_freq = aSinFreq * temp;

	}
	aMod.setFreq(mod_freq);
}

void lockKnobs() {
	for (int i = 0; i < 4; i++) {
		lockAnchor[i] = mozziRaw[i];
		knobLock[i] = true;
	}
}
