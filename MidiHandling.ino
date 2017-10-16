void HandleNoteOn(byte channel, byte note, byte velocity) {
	notecounter++;
	noteFreq = mtof(float(note));
	aSinFreq = noteFreq;// +offsetFreq;
	aSin.setFreq(aSinFreq);
	envelope.noteOn();
	FMenvelope.noteOn();
	digitalWrite(LED, HIGH);
}

void HandleNoteOff(byte channel, byte note, byte velocity) {
	notecounter--;
	envelope.noteOff();
	FMenvelope.noteOff();
	digitalWrite(LED, LOW);
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