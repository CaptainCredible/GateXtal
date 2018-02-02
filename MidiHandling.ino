void writeLED(bool state) {
	if (!syncPulse) {
		digitalWrite(LED, state);
		ignoreLEDstate = state;
	}
}

void HandleDINNoteOn(byte channel, byte note, byte velocity) {
	HandleNoteOn(note, velocity);
	if (pageState == 3 && buttStates[BUTTON2]) {
		noteToWrite = note;
		writeToSeq();
	}
}

void HandleNoteOn(byte note, byte velocity) {
	if (note != 0) {   //if not zero
		noteIsOn = true;
		noteFreq = mtof(float(note));
		aSinFreq = noteFreq;
		aSin.setFreq(aSinFreq);
		envelope.noteOn();
		writeLED(true);
		lastNote = note;
	}
}

void HandleDINNoteOff(byte channel, byte note, byte velocity) {
	HandleNoteOff(note, velocity);
}

void HandleNoteOff(byte note, byte velocity) {
	noteIsOn = false;
	if (note == lastNote) { //only turn voice off if it was the last note to be pushed that was released
		envelope.noteOff();
		writeLED(false);
	}
}

void HandleMIDIClock() {

}



void usbmidiprocessing(){

	while (MIDIUSB.available() > 0) {
		MIDIEvent e = MIDIUSB.read();
		
		// IF NOTE ON WITH VELOCITY GREATER THAN ZERO
		if ((e.type == NOTEON) && (e.m3 > 0)) {
			//jitterfreq = 0;
			HandleNoteOn(e.m2, e.m3);
			if (pageState == 3 && buttStates[BUTTON2]) {
				noteToWrite = e.m2;
				writeToSeq();
			}
		}
		// IF USB NOTE OFF
		else if (e.type == NOTEOFF) {
			HandleNoteOff(e.m2, e.m3);
		}
		// IF NOTE ON W/ ZERO VELOCITY
		else if ((e.type == NOTEON) && (e.m3 == 0)) {
			if (!internalClockSelect) {
				HandleNoteOff(e.m2, e.m3);
			}
		}
		else if (e.type == TICK) {
			if (!internalClockSelect) {
				handleMidiClockTicks();
			}
			if (e.m1 == 252) {
				HandleNoteOff(sequence[seqCurrentStep], 0);
			}
		}
		else if (e.type == RESTART) {
			if (!internalClockSelect) {
				resetSeq();
			}
		}
		
	}
}