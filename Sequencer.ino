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

	if (seqIncrement > seqNoteLength && notecounter > 0) {
		HandleNoteOff(1, sequence[seqCurrentStep], 0);
	}

	if (seqIncrement > seqTempo) {
		playNextStep();
		seqIncrement = 0;
		Serial.print(" -  TEMPO = ");
		Serial.println(seqTempo);

	}

}

void seqPlayStep(byte step) {
	if (sequence[step]) {									//if the step is not a zero
		HandleNoteOff(1, sequence[(step - 1) % 16], 0);		//turn off prev note
		//Serial.print("turned off ");
		//Serial.println(sequence[(step - 1) % 16]);
		HandleNoteOn(1, sequence[step], 127);				//turn on next note
		arcadeNote = sequence[step];
		//Serial.print("turned on ");
		//Serial.print(sequence[step]);

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
		//Serial.println(noteToWrite);
		//Serial.println(refreshWriteNotePing);

	}

}

void seqCheckButts() {
	if (buttStates[BUTTON2] && !oldButtStates[BUTTON2]) { //IF BUTTON 2 was just pressed
		//Serial.println("ping");							//debug
		sequence[seqCurrentStep] = noteToWrite;			//write our note here
		HandleNoteOff(1, noteToWrite, 0);				//
		seqCurrentStep++;
		seqCurrentStep = seqCurrentStep % seqLength;
		oldButtStates[BUTTON2] = buttStates[BUTTON2];
	}
	else if (!buttStates[BUTTON2] && oldButtStates[BUTTON2]) {
		oldButtStates[BUTTON2] = buttStates[BUTTON2];
	}

	else if (buttStates[BUTTON3] && !oldButtStates[BUTTON3]) {
		sequence[seqCurrentStep] = 0;
		seqCurrentStep++;
		seqCurrentStep = seqCurrentStep % seqLength;
		oldButtStates[BUTTON3] = buttStates[BUTTON3];
	}
	else if (!buttStates[BUTTON3] && oldButtStates[BUTTON3]) {
		oldButtStates[BUTTON3] = buttStates[BUTTON3];
	}
	

	else if (buttStates[BUTTON1] && !oldButtStates[BUTTON1]) {
		//playPrevStep();
		oldButtStates[BUTTON1] = buttStates[BUTTON1];
	} 
	else if (!buttStates[BUTTON1] && oldButtStates[BUTTON1]) {
		//HandleNoteOff(1,sequence[seqCurrentStep],0);
		oldButtStates[BUTTON1] = buttStates[BUTTON1];
	}



	}


