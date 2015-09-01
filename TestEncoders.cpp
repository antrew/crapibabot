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
#include "Encoder.h"
#include "Motor.h"

int pwm_divisor = 40;
int pwm_range = 1000;

Motor* motorLeft;
Motor* motorRight;
Encoder* encoderLeft;
Encoder* encoderRight;

void shutdown(void) {
	delete motorLeft;
	delete motorRight;
	delete encoderLeft;
	delete encoderRight;
}

void interruptHandlerEncoderLeft(void) {
	int otherPinValue = digitalRead(PORT_NUMBER_ENCODER_LEFT_B);
	encoderLeft->updateCounters(otherPinValue);
}

void interruptHandlerEncoderRight(void) {
	int otherPinValue = digitalRead(PORT_NUMBER_ENCODER_RIGHT_B);
	encoderRight->updateCounters(otherPinValue);
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
	sigaction(SIGINT, &action, NULL);

	//encoder LEFT
	encoderLeft = new Encoder("encoder left", PORT_NUMBER_ENCODER_LEFT_A, PORT_NUMBER_ENCODER_LEFT_B, 12, 8.6, 9.7);

	//encoder RIGHT
	encoderRight = new Encoder("encoder right", PORT_NUMBER_ENCODER_RIGHT_A, PORT_NUMBER_ENCODER_RIGHT_B, 12, 8.6, 9.7);

	//motor LEFT
	motorLeft = new Motor("motor left", PORT_NUMBER_MOTOR_LEFT_PWM,
		PORT_NUMBER_MOTOR_LEFT_BACKWARD, PORT_NUMBER_MOTOR_LEFT_FORWARD, pwm_range);

	//motor RIGHT
	motorRight = new Motor("motor right", PORT_NUMBER_MOTOR_RIGHT_PWM,
		PORT_NUMBER_MOTOR_RIGHT_BACKWARD, PORT_NUMBER_MOTOR_RIGHT_FORWARD, pwm_range);

	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(pwm_range);
	pwmSetClock(pwm_divisor);

	// pwmFrequency in Hz = 19.2 MHz / pwmClock / pwmRange
	int frequency = 19200000 / pwm_divisor / pwm_range;

	printf("frequency=%d Hz (divisor=%d, range=%d)\n", frequency, pwm_divisor,
			pwm_range);

	wiringPiISR(encoderLeft->getPortA(), INT_EDGE_RISING, &interruptHandlerEncoderLeft);
	wiringPiISR(encoderRight->getPortA(), INT_EDGE_RISING, &interruptHandlerEncoderRight);
}

void doOneWheelRevolution(int powerPercentMotorLeft, int powerPercentMotorRight, int ticksPerRevolution, float gearRatio){
	digitalWrite(PORT_NUMBER_MOTOR_LEFT_FORWARD, HIGH);
	digitalWrite(PORT_NUMBER_MOTOR_RIGHT_FORWARD, HIGH);

	bool oneRevolutionWheelLeftCompleted = false;
	bool oneRevolutionWheelRightCompleted = false;

	do{
		motorLeft->setPower(powerPercentMotorLeft);
		motorRight->setPower(powerPercentMotorRight);

		delay(50);

		long encoderLeftCounter = encoderLeft->getCounter();
		long encoderRightCounter = encoderRight->getCounter();

		if(encoderLeftCounter >= (ticksPerRevolution * gearRatio)){
			oneRevolutionWheelLeftCompleted = true;
			powerPercentMotorLeft = 0;
		}

		if(encoderRightCounter >= (ticksPerRevolution * gearRatio)){
			oneRevolutionWheelRightCompleted = true;
			powerPercentMotorRight = 0;
		}

		printf("powerPercentMotorLeft=%d\t counterLeft=%ld\t stepsLeft=%ld\t distanceLeft=%ld [cm]\t powerPercentMotorRight=%d\t counterRight=%ld\t stepsRight=%ld\t distanceRight=%ld [cm]\t\n", powerPercentMotorLeft,
				encoderLeftCounter, encoderLeft->getSteps(), encoderLeft->getDistance(), powerPercentMotorRight, encoderRightCounter, encoderRight->getSteps(), encoderRight->getDistance());

	}while((oneRevolutionWheelLeftCompleted == false) || (oneRevolutionWheelRightCompleted == false));
}

void rotateWhileIncreasingPowerPercentage(int percentage = 0){
	while (percentage < 35) {
		int pwm_value = pwm_range * percentage / 100;
		digitalWrite(PORT_NUMBER_MOTOR_LEFT_FORWARD, HIGH);
		pwmWrite(PORT_NUMBER_MOTOR_LEFT_PWM, pwm_value);

		digitalWrite(PORT_NUMBER_MOTOR_RIGHT_FORWARD, HIGH);
		pwmWrite(PORT_NUMBER_MOTOR_RIGHT_PWM, pwm_value);

		delay(1000);

		printf("percentage=%d\t counterLeft=%ld\t stepsLeft=%ld\t distanceLeft=%ld [cm]\t counterRight=%ld\t stepsRight=%ld\t distanceRight=%ld [cm]\t\n", percentage,
				encoderLeft->getCounter(), encoderLeft->getSteps(), encoderLeft->getDistance(), encoderRight->getCounter(), encoderRight->getSteps(), encoderRight->getDistance());

		percentage += 1;
	}

	digitalWrite(PORT_NUMBER_MOTOR_LEFT_FORWARD, LOW);
	digitalWrite(PORT_NUMBER_MOTOR_RIGHT_FORWARD, LOW);

	while (percentage > 0) {
		int pwm_value = pwm_range * percentage / 100;
		digitalWrite(PORT_NUMBER_MOTOR_LEFT_BACKWARD, HIGH);
		pwmWrite(PORT_NUMBER_MOTOR_LEFT_PWM, pwm_value);

		digitalWrite(PORT_NUMBER_MOTOR_RIGHT_BACKWARD, HIGH);
		pwmWrite(PORT_NUMBER_MOTOR_RIGHT_PWM, pwm_value);

		delay(1000);

		printf("percentage=%d\t counterLeft=%ld\t stepsLeft=%ld\t distanceLeft=%ld [cm]\t counterRight=%ld\t stepsRight=%ld\t distanceRight=%ld [cm]\t\n", percentage,
						encoderLeft->getCounter(), encoderLeft->getSteps(), encoderLeft->getDistance(), encoderRight->getCounter(), encoderRight->getSteps(), encoderRight->getDistance());

		percentage -= 1;
	}
}

int main(void) {
	init();

	doOneWheelRevolution(14, 15, 12, 9.7);

	encoderLeft->resetCounters();
	encoderRight->resetCounters();

	rotateWhileIncreasingPowerPercentage();

	shutdown();
	return 0;
}
