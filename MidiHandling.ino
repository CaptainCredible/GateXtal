void HandleNoteOn(byte channel, byte note, byte velocity) {
	notecounter++;
	noteFreq = mtof(float(note));
	aSinFreq = noteFreq;// +lfoOutput;
	aSin.setFreq(aSinFreq);
	envelope.noteOn();
	//MODenvelope.noteOn();
	digitalWrite(LED, HIGH);
	lastNote = note;
}

void HandleNoteOff(byte channel, byte note, byte velocity) {
	notecounter--;
	if (note == lastNote) { //only turn voice off if it was the last note to be pushed that was released
		envelope.noteOff();
		//MODenvelope.noteOff();
		digitalWrite(LED, LOW);
	}
}

void usbmidiprocessing()
{
	while (MIDIUSB.available() > 0) {
		MIDIEvent e = MIDIUSB.read();

		// IF NOTE ON WITH VELOCITY GREATER THAN ZERO
		if ((e.type == NOTEON) && (e.m3 > 0)) {
			jitterfreq = 0;
			digitalWrite(LED, HIGH);
			HandleNoteOn(e.m1, e.m2, e.m3);
			//Serial.println(notecounter);

		}

		// IF NOTE ON W/ ZERO VELOCITY
		if ((e.type == NOTEON) && (e.m3 == 0)) {
			HandleNoteOff(e.m1, e.m2, e.m3);
			//Serial.println(notecounter);


		}
		// IF USB NOTE OFF
		if (e.type == NOTEOFF) {
			HandleNoteOff(e.m1, e.m2, e.m3);
			//Serial.println(notecounter);
		}
	}
}