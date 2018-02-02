// byte sequence[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };



void handleMidiClockTicks() {
	midiClockTicks++;
	midiClockTicks = midiClockTicks % midiClockStepSize; 
	if (midiClockTicks == 1) {
		playNextStep();

	}
	else if (midiClockTicks > midiSeqNoteLength && noteIsOn) {  // if we have passed midinotelength and a note is one
		HandleNoteOff(sequence[seqCurrentStep], 0);
	}
}

void handleSequencer() {
	if (internalClockSelect) { //if seqClockTyoe is 0, aka internal
		handleSeqClock();

	}
	else {				//if seq clock is external
		if (digitalRead(LED) && !syncPulse && !ignoreLEDstate) {
			//Serial.println("RECEIVED PULSE");
			syncPulse = true;
			playNextStep();
		}
		else if (!digitalRead(LED) && syncPulse) {
			syncPulse = false;
			HandleNoteOff(sequence[seqCurrentStep], 0);
		}
	}
}

void resetSeq() {
	seqCurrentStep = seqLength-1;
	midiClockTicks = 0;
}

void playPrevStep() {
	seqCurrentStep--;
	seqCurrentStep = seqCurrentStep % seqLength;
	seqPlayStep(seqCurrentStep);
}

void playNextStep() {
	seqCurrentStep++;
	//Serial.print("STEP = ");
	//Serial.print(seqCurrentStep);
	//Serial.print(" Seq LENGTH = ");
	//Serial.println(seqLength);

	seqCurrentStep = seqCurrentStep % seqLength;
	seqPlayStep(seqCurrentStep);
}

void handleSeqClock() {  // handle internal sequencerclock
	seqIncrement++;
	//Serial.print("#");

	if (seqIncrement > seqNoteLength && noteIsOn) {			//turn note off after it reached its length
		HandleNoteOff(sequence[seqCurrentStep], 0);
	}

	if (seqIncrement > seqTempo) {							//go to next step after prev step was nailored
		playNextStep();
		//Serial.println(" CLICK");
		seqIncrement = 0;

	}

}

void seqPlayStep(byte step) {
	if (sequence[step]) {									//if the step is not a zero
		if (noteIsOn) {
			HandleNoteOff(sequence[(step - 1) % 16], 0);		//turn off prev note
		}
		HandleNoteOn(sequence[step], 127);				//turn on next note
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
		HandleNoteOff(prevNoteSelect, 0);
		noteToWrite = noteSelect + octOffset;
		HandleNoteOn(noteToWrite, 127);
		prevNoteSelect = noteSelect;
		refreshWriteNotePing = false;
	}

}
void writeToSeq() {
	sequence[seqCurrentStep] = noteToWrite;
	//Serial.print("wrote note ");
	//Serial.print(noteToWrite);
	//Serial.print("  on step ");
	//Serial.print(seqCurrentStep);
	seqLength++; //extend sequence length
	seqLength = seqLength % sizeof(sequence); // dont allow sequence to be longer than 64 steps
	seqCurrentStep++;
	seqCurrentStep = seqCurrentStep % sizeof(sequence); // same for current step obviously

	//Serial.print("  length is  ");
	//Serial.println(seqLength);
}

bool EEPROMwriting = false;//flag to ignore all buttons until every button is released
bool oldEEPROMwriting = false;// check if it was just turned off
void seqCheckButts() {
	if (EEPROMwriting) {
		if (!(buttStates[BUTTON1] || buttStates[BUTTON2] || buttStates[BUTTON3])) {
			EEPROMwriting = false;
			//Serial.println("EEPROM WRITING OVER");
			allLedsOff();
		}
	}
	else {

		//BUTTON1

		if (buttStates[BUTTON1] && !oldButtStates[BUTTON1]) {
			oldButtStates[BUTTON1] = buttStates[BUTTON1];
			if (buttStates[BUTTON2] && buttStates[BUTTON3]) {
				//Serial.println("EEPROM WRITE");
				writeSeqToEeprom();
				EEPROMwriting = true; //flag to ignore all buttons until every button is released
			}
			else {
				internalClockSelect = !internalClockSelect;						//toggle internalClockSelect
				if (!internalClockSelect) {								//if we land on 0 (ext clock)
					HandleNoteOff(sequence[seqCurrentStep], 127);	//stop any note that might be on
					//Serial.print("STOPPED");						//DEBUG
					seqIncrement = seqTempo;						//reset seq incrementor so it starts
					pinMode(LED, INPUT);							//change LED to input for sync listening
				}
				else {												//if we landed on 1 (int clock);
					pinMode(LED, OUTPUT);							//enable bright led
					seqCurrentStep = seqLength - 1;
					seqIncrement = seqTempo;
				}
			}
		}
		else if (!buttStates[BUTTON1] && oldButtStates[BUTTON1]) {



			oldButtStates[BUTTON1] = buttStates[BUTTON1];
		}

		//BUTTON 2

		else if (buttStates[BUTTON2] && !oldButtStates[BUTTON2]) { //IF BUTTON 2 was just pressed
			oldButtStates[BUTTON2] = buttStates[BUTTON2];
			if (buttStates[BUTTON3]) {
				//dont do nuttin cos we are preparing for 3 button EEPROM WRITE
			}
			else {
				writeMode = true;
				seqLength = 0; //reset sequence length
				seqCurrentStep = 0; //reset sequence
			}
		}
		else if (!buttStates[BUTTON2] && oldButtStates[BUTTON2]) { // when write button is released!!!
			//seqLength = seqCurrentStep;
			seqCurrentStep = seqLength - 1;
			writeMode = false;
			HandleNoteOff(noteToWrite, 127);
			oldButtStates[BUTTON2] = buttStates[BUTTON2];
		}


		//BUTTON 3
		else if (buttStates[BUTTON3] && !oldButtStates[BUTTON3]) {
			oldButtStates[BUTTON3] = buttStates[BUTTON3];

			if (writeMode) {
				noteToWrite = 0;
				writeToSeq();
				//Serial.println("wrote a pause");
			}
		}
		else if (!buttStates[BUTTON3] && oldButtStates[BUTTON3]) {
			oldButtStates[BUTTON3] = buttStates[BUTTON3];
		}
	}


}

void writeSeqToEeprom() {
	allLedsOn();
	for (byte i = 0; i < seqLength; i++) {
		EEPROM.write(i, sequence[i]);
		//Serial.print("Step ");
		//Serial.print(i);
		//Serial.print(" = ");
		//Serial.println(sequence[i]);
	}
	EEPROM.write(120, seqLength);

	EEPROM.write(100, 123); //magic flag so we can tell if EEPROM contains a sequence or mumbo jumbo.

}

void readSeqFromEeprom() {

	seqLength = EEPROM.read(120); //we need to read seqlength first so we now how far to read EEPROM

	for (int i = 0; i < seqLength; i++) {
		sequence[i] = EEPROM.read(i);
		//Serial.print("Step ");
		//Serial.print(i);
		//Serial.print(" = ");
		//Serial.println(sequence[i]);
	}

}

void allLedsOn() {
	for (byte i = 0; i < 3; i++) {
		digitalWrite(LEDS[i], HIGH);
	}
}

void allLedsOff() {
	for (byte i = 0; i < 3; i++) {
		digitalWrite(LEDS[i], LOW);
	}
}