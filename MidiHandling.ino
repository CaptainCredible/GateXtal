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
		int octedNote = constrain(note + (octTranspose * 12), 0, 127);
		noteIsOn = true;
		noteFreq = mtof(float(octedNote));
		aSinFreq = noteFreq;
		aSin.setFreq(aSinFreq);
		envelope.noteOn();
		writeLED(true);
		lastNote = note;
	}
}

void legato(byte note) {
	noteFreq = mtof(float(note));
	aSinFreq = noteFreq;
	aSin.setFreq(aSinFreq);
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

void handleMIDIClock() {
	if (!internalClockSelect) {
		handleMidiClockTicks();
	}
}

void handleMIDIClockStart(){
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



void usbmidiprocessing(){
	/*
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
				if (!internalClockSelect) {
					HandleNoteOff(sequence[seqCurrentStep], 0);
					midiClockRunning = false;
				}
			}
		}
		else if (e.type == RESTART) {
			if (!internalClockSelect) {
				resetSeq();
			}
		}
		
	}
	*/
}