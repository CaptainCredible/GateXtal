void setWaveForm(byte waveNumber) {
	switch (waveNumber) {
	case 0:
		if(!buttStates[BUTTON3]){
		aSin.setTable(SIN2048_DATA);
		}
		//Serial.print("waveform = SINE");
		aMod.setTable(SIN2048_DATA);
		//LFO.setTable(BROWNNOISE8192_DATA);

		break;

	case 1:
		if (!buttStates[BUTTON3]) {
			aSin.setTable(TRIANGLE2048_DATA);
		}
		//Serial.print("waveform = TRIANGLE");
		aMod.setTable(TRIANGLE2048_DATA);
		break;

	case 2:
		if (!buttStates[BUTTON3]) {
			aSin.setTable(SAW2048_DATA);
		}
		//Serial.print("waveform = SAW");
		aMod.setTable(SAW2048_DATA);
		break;

	case 3:
		if (!buttStates[BUTTON3]) {
			aSin.setTable(SQUARE_ANALOGUE512_DATA);
		}
		//Serial.print("waveform = SQUARE");
		aMod.setTable(SQUARE_ANALOGUE512_DATA);
		break;

	}
}