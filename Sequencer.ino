byte sequence[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
unsigned int seqIncrement = 0;
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
	seqCurrentStep % seqLength;
	seqPlayStep(seqCurrentStep);
}

void handleSeqClock() {  // handle internal sequencerclock
	seqIncrement++;
	if (seqIncrement > tempo) {
		playNextStep();
		seqIncrement = 0;
	}

}

void seqPlayStep(byte step) {
	if (sequence[step]) {									//if the step is not a zero
		HandleNoteOff(1, sequence[(step - 1) % 16], 0);		//turn off prev note
		HandleNoteOn(1, sequence[step], 127);				//turn on next note
	}
}

void seqClearStep(byte step) {
	sequence[step] = 0;
}

void seqWriteStep(byte step, byte note) {
	sequence[step] = note;
}
