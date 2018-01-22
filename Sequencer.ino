// byte sequence[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
byte sequence[16] = { 40,44,47,40,44,48,40,44,47,40,44,48,40,44,47,51};
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
	if (!seqClockType) { //if seqClockTyoe is 0, aka internal
		handleSeqClock();
	}
}

void playNextStep() {
	seqCurrentStep++;
	seqCurrentStep = seqCurrentStep % seqLength;
	seqPlayStep(seqCurrentStep);

}

void handleSeqClock() {  // handle internal sequencerclock
	seqIncrement++;
	if (seqIncrement > seqTempo) {
		playNextStep();
		seqIncrement = 0;
	}

}

void seqPlayStep(byte step) {
	if (sequence[step]) {									//if the step is not a zero
		HandleNoteOff(1, sequence[(step - 1) % 16], 0);		//turn off prev note
		Serial.print("turned off ");
		Serial.println(sequence[(step - 1) % 16]);
		HandleNoteOn(1, sequence[step], 127);				//turn on next note
		arcadeNote = sequence[step];
		Serial.print("turned on ");
		Serial.print(sequence[step]);

	}
}

void seqClearStep(byte step) {
	sequence[step] = 0;
}

void seqWriteStep(byte step, byte note) {
	sequence[step] = note;
}

//int repeater = 0;
// byte oldthisNote = 0;
void setWriteNote(byte thisNote) {
	if (thisNote != noteSelect || refreshWriteNotePing) { //If we are on a new note
		//Serial.print(repeater);
		//Serial.print(" - ");
		//repeater++;
		byte octOffset = writeOctSelect * 12;
		noteSelect = thisNote;// +octOffset;
		HandleNoteOff(1, prevNoteSelect, 0);
		HandleNoteOn(1,noteSelect + octOffset,127);
		//HandleNoteOff(1, noteSelect + octOffset, 127);

		//Serial.print(refreshWriteNotePing);
		//Serial.print(" - ");
		//Serial.print(noteSelect);
		//Serial.print(" - ");
		//Serial.println(prevNoteSelect);
		
		prevNoteSelect = noteSelect;
		refreshWriteNotePing = false;
	}
}

