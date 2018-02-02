void setWaveForm(byte waveNumber) {
	switch (waveNumber) {
	case 0:
		if(!buttStates[BUTTON3]){
		//aSIN.setTable(SIN2048_DATA);
			aSin.setTable(SIN1024_DATA);
		}
		else {
			//aMod.setTable(SIN2048_DATA);
			aMod.setTable(SIN1024_DATA);
		}
		break;

	case 1:
		if (!buttStates[BUTTON3]) {
			aSin.setTable(TRIANGLE2048_DATA);
		} else {
		aMod.setTable(TRIANGLE2048_DATA);
		}
		break;

	case 2:
		if (!buttStates[BUTTON3]) {
			aSin.setTable(SAW2048_DATA);
		} else {
		aMod.setTable(SAW2048_DATA);
		}
		break;

	case 3:
		if (!buttStates[BUTTON3]) {
			aSin.setTable(SQUARE_ANALOGUE512_DATA);
		} else {
		aMod.setTable(SQUARE_ANALOGUE512_DATA);
		}
		break;

	}
}