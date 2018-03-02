void debug() {
	debugPort.print(" val = ");
	debugPort.print(globalVal);
	debugPort.print(" mod_ratio = ");
	debugPort.print(mod_ratio);
	debugPort.print(" maxLevel = ");
	debugPort.print(outPuTTY);
	debugPort.print(" UPDATE - ");
	debugPort.println(count);

	count++;
}


void countTo(int number) {
	for (int i = 0; i < number; i++) {
		debugPort.println(i);
		//delay(10);
	}
}
