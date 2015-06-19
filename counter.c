/*
 * test.c
 *
 *  Created on: 19.06.2015
 *      Author: andy
 */

#include <wiringPi.h>

int pinA = 19;
int pinB = 26;
int counter = 0;

void interruptHandler(void) {
	int pinAvalue = digitalRead(pinA);
	if (pinAvalue) {
		counter++;
	} else {
		counter--;
	}
}

int main(void) {

	wiringPiSetupGpio();

	pinMode(pinA, INPUT);
	pinMode(pinB, INPUT);

	wiringPiISR(pinB, INT_EDGE_RISING, &interruptHandler);

	for (;;) {
		printf("%d\n", counter);
		delay(500);
	}
	return 0;
}
