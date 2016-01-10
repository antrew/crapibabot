/*
 * test.c
 *
 *  Created on: 19.06.2015
 *      Author: andy
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "wiringPi.h"
#include "ports.h"

int pwm_divisor = 40;
int pwm_range = 1000;

int counter = 0;

void shutdown(void) {
	pwmWrite(port_motor_left_pwm, 0);
	digitalWrite(port_motor_left_backward, LOW);
	digitalWrite(port_motor_left_forward, LOW);
}

void interruptHandler(void) {
	int otherPinValue = digitalRead(port_encoder_left_b);
	if (otherPinValue) {
		counter++;
	} else {
		counter--;
	}
}

void term(int signum) {
	shutdown();
	exit(1);
}

void init(void) {
	wiringPiSetupGpio();

	// SIGTERM handler
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = term;
	sigaction(SIGTERM, &action, NULL);

	pinMode(port_encoder_left_a, INPUT);
	pinMode(port_encoder_left_b, INPUT);

	pinMode(port_motor_left_pwm, PWM_OUTPUT);
	pinMode(port_motor_left_backward, OUTPUT);
	pinMode(port_motor_left_forward, OUTPUT);

	digitalWrite(port_motor_left_backward, LOW);
	digitalWrite(port_motor_left_forward, LOW);

	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(pwm_range);
	pwmSetClock(pwm_divisor);

	// pwmFrequency in Hz = 19.2 MHz / pwmClock / pwmRange
	int frequency = 19200000 / pwm_divisor / pwm_range;

	printf("frequency=%d Hz (divisor=%d, range=%d)\n", frequency, pwm_divisor,
			pwm_range);

	wiringPiISR(port_encoder_left_a, INT_EDGE_RISING, &interruptHandler);
}

int main(void) {
	init();

	int percentage = 0;
	int previousCounter = 0;
	int steps;
	while (percentage < 100) {
		int pwm_value = pwm_range * percentage / 100;
		digitalWrite(port_motor_left_forward, HIGH);
		pwmWrite(port_motor_left_pwm, pwm_value);

		delay(1000);

		steps = counter - previousCounter;
		previousCounter = counter;

		printf("percentage=%d counterleft=%d stepsleft=%d\n", percentage,
				counter, steps);

		percentage += 1;
	}

	shutdown();
	return 0;
}
