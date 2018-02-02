// byte sequence[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };


int temp = 0;
void handleMidiClockTicks() {
	midiClockTicks++;
	midiClockTicks = midiClockTicks % 24;
	//Serial.println(midiClockTicks);
	if (midiClockTicks == 1) {
		playNextStep();
		temp++;
		Serial.print("TICK - ");
		Serial.println(temp);
	}
	else if (midiClockTicks > midiSeqNoteLength && noteIsOn) {  // if we have passed midinotelength and a note is one
		HandleNoteOff(1, sequence[seqCurrentStep], 0);
	}
}

void handleSequencer() {
	if (seqClockType == 1) { //if seqClockTyoe is 0, aka internal
		handleSeqClock();
	}
}

void resetSeq() {
	seqCurrentStep = 0;
}

void playPrevStep() {
	seqCurrentStep--;
	seqCurrentStep = seqCurrentStep % seqLength;
	seqPlayStep(seqCurrentStep);
}

void playNextStep() {
	Serial.print("STEP = ");
	Serial.print(seqCurrentStep);
	Serial.print(" Seq LENGTH = ");
	Serial.println(seqLength);
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
void writeToSeq() {
	sequence[seqCurrentStep] = noteToWrite;
	seqLength++; //extend sequence length
	seqLength = seqLength % 64; // dont allow sequence to be longer than 64 steps
	seqCurrentStep++; 
	seqCurrentStep = seqCurrentStep % 64; // same for current step obviously
	Serial.print("wrote note ");
	Serial.print(noteToWrite);
	Serial.print("  length is  ");
	Serial.println(seqLength);
}

void seqCheckButts() {
	if (buttStates[BUTTON2] && !oldButtStates[BUTTON2]) { //IF BUTTON 2 was just pressed
		oldButtStates[BUTTON2] = buttStates[BUTTON2];
		if (buttStates[BUTTON1]) {
			//dont write a note cos we are preparing for 3 button EEPROM WRITE
		}
		else { 
			seqLength = 0; //reset sequence length
			seqCurrentStep = 0; //reset sequence
		}
	}
	else if (!buttStates[BUTTON2] && oldButtStates[BUTTON2]) { // when write button is released!!!
		seqLength = seqCurrentStep;
		seqCurrentStep = seqLength -1;
		//writeToSeq();
		HandleNoteOff(1, noteToWrite, 127);
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
	for (byte i = 0; i < 4; i++) {
		digitalWrite(LEDS[i], HIGH);
	}
	for (byte i = 0; i < 16; i++) {
		EEPROM.write(i, sequence[i]);
	}
	EEPROM.write(120, seqLength);
	EEPROM.write(100, 123); //magic flag so we can tell if EEPROM contains a sequence or mumbo jumbo.
	for (byte i = 0; i < 3; i++) {
		digitalWrite(LEDS[i], LOW);
	}

}

void readSeqFromEeprom() {
	for (int i = 0; i < 16; i++) {
		sequence[i] = EEPROM.read(i);
	}
	seqLength = EEPROM.read(120);
}