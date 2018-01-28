// byte sequence[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
byte sequence[16] = { 40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51 };
unsigned int seqIncrement = 0; //counter to keep track of when next step should come
byte seqCurrentStep = 0;
byte midiClockTicks = 0;

void handleMidiClockTicks() {
	midiClockTicks++;
	midiClockTicks = midiClockTicks % (24 / midiClockDivider);
	if (midiClockTicks == 1) {
		playNextStep();
	}
}

void handleSequencer() {
	if (seqClockType == 1) { //if seqClockTyoe is 0, aka internal
		handleSeqClock();
	}
}

void playPrevStep() {
	seqCurrentStep--;
	seqCurrentStep = seqCurrentStep % seqLength;
	seqPlayStep(seqCurrentStep);
}

void playNextStep() {
	seqCurrentStep++;
	seqCurrentStep = seqCurrentStep % seqLength;
	seqPlayStep(seqCurrentStep);
}

void handleSeqClock() {  // handle internal sequencerclock
	seqIncrement++;

	if (seqIncrement > seqNoteLength && noteIsOn) {
		HandleNoteOff(1, sequence[seqCurrentStep], 0);
	}

	if (seqIncrement > seqTempo) {
		playNextStep();
		seqIncrement = 0;

	}

}

void seqPlayStep(byte step) {
	if (sequence[step]) {									//if the step is not a zero
		if (noteIsOn) {
			HandleNoteOff(1, sequence[(step - 1) % 16], 0);		//turn off prev note
		}
		HandleNoteOn(1, sequence[step], 127);				//turn on next note
		arcadeNote = sequence[step];

	}
}

//void seqClearStep(byte step) {
//	sequence[step] = 0;
//}

//void seqWriteStep(byte step, byte note) {
//	sequence[step] = note;
//}
byte noteToWrite = 0;
byte octOffset = 0;

void setWriteNote(byte thisNote) {
	if (thisNote != noteSelect || refreshWriteNotePing) { //If we are on a new note

		octOffset = writeOctSelect * 12;
		noteSelect = thisNote;// +octOffset;
		HandleNoteOff(1, prevNoteSelect, 0);
		noteToWrite = noteSelect + octOffset;
		HandleNoteOn(1, noteToWrite, 127);
		prevNoteSelect = noteSelect;
		refreshWriteNotePing = false;


	}

}

void seqCheckButts() {
	if (buttStates[BUTTON2] && !oldButtStates[BUTTON2]) { //IF BUTTON 2 was just pressed
		oldButtStates[BUTTON2] = buttStates[BUTTON2];
		if (buttStates[BUTTON1]) {
			//dont write a note cos we are preparing for 3 button EEPROM WRITE
		}
		else {
			sequence[seqCurrentStep] = noteToWrite;			//write our note here
			HandleNoteOff(1, noteToWrite, 0);				//
			seqCurrentStep++;
			seqCurrentStep = seqCurrentStep % seqLength;
		}
	}
	else if (!buttStates[BUTTON2] && oldButtStates[BUTTON2]) {
		oldButtStates[BUTTON2] = buttStates[BUTTON2];
	}

	else if (buttStates[BUTTON3] && !oldButtStates[BUTTON3]) {
		oldButtStates[BUTTON3] = buttStates[BUTTON3];
		if (buttStates[BUTTON1] && buttStates[BUTTON2]) {
			Serial.println("EEPROM WRITE");
			writeSeqToEeprom();
		}
		else {
			sequence[seqCurrentStep] = 0;
			seqCurrentStep++;
			seqCurrentStep = seqCurrentStep % seqLength;
		}
	}
	else if (!buttStates[BUTTON3] && oldButtStates[BUTTON3]) {
		oldButtStates[BUTTON3] = buttStates[BUTTON3];
	}


	else if (buttStates[BUTTON1] && !oldButtStates[BUTTON1]) {
		oldButtStates[BUTTON1] = buttStates[BUTTON1];
	}
	else if (!buttStates[BUTTON1] && oldButtStates[BUTTON1]) {
		//HandleNoteOff(1,sequence[seqCurrentStep],0);
		oldButtStates[BUTTON1] = buttStates[BUTTON1];
	}



}

void writeSeqToEeprom() {
	for (byte i = 0; i < 16; i++) {
		for (byte i = 0; i < 4; i++) {
			digitalWrite(LEDS[i], HIGH);
		}
		EEPROM.write(i, sequence[i]);
		for (byte i = 0; i < 3; i++) {
			digitalWrite(LEDS[i], LOW);
		}
	}
	EEPROM.write(100, 123); //magic flag so we can tell if EEPROM contains a sequence or mumbo jumbo.
}

void readSeqFromEeprom() {
	for (int i = 0; i < 16; i++) {
		sequence[i] = EEPROM.read(i);
	}
	EEPROM.write(100, 123); //magic flag so we can tell if EEPROM contains a sequence or mumbo jumbo.
}