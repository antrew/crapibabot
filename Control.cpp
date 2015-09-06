/*
 * Control.cpp
 *
 *  Created on: Aug 2, 2015
 *      Author: kaladze
 */

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <inttypes.h>
#include "I2Cdev.h"
#include "wiringPi.h"
#include "ports.h"
#include "MyMPU6050.h"
#include "ComplementaryFilter.h"
#include "Motor.h"
#include "Encoder.h"

const int pwmDivisor = 40;
const int pwmRange = 1000;

const int ticksPerRevolution = 12;
const float wheelDiameter = 8.6;
const float gearRatio = 9.7;

float Kp;
float Ki;
float Kd;
float set_point;
float Kangle;

float integral_error;
float last_error;


int countCalibrationAngles = 0;
float meanCalibrationAngle = 0.0;
long double sumCalibrationAngle = 0.0;
float calibrationAngle = 0.0;
int calibrationCompleted = 0;
int calibrationStartTime = 0;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MyMPU6050 mpu;
ComplementaryFilter complementaryFilter;
Motor* motorLeft;
Motor* motorRight;
Encoder* encoderLeft;
Encoder* encoderRight;

void shutdownMotors(void) {
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
	shutdownMotors();
	exit(1);
}

// ================================================================
// ===         INITIAL SENSORS (acc & gyro) SETUP               ===
// ================================================================

void setupSensors() {
	// initialize device
	printf("Initializing I2C devices...\n");
	mpu.initialize();

	// verify connection
	printf("Testing device connections...\n");
	// TODO handle the error (stop the program with a meaningful message for the user)
	printf(
			mpu.testConnection() ?
					"MPU6050 connection successful\n" :
					"MPU6050 connection failed\n");

}

// ================================================================
// ===                 INITIAL MOTORS SETUP                     ===
// ================================================================

void setupMotors() {
	wiringPiSetupGpio();

	// SIGTERM handler
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = term;
	sigaction(SIGINT, &action, NULL);

	//encoder LEFT
	encoderLeft = new Encoder("encoder left", PORT_NUMBER_ENCODER_LEFT_A, PORT_NUMBER_ENCODER_LEFT_B, ticksPerRevolution, wheelDiameter, gearRatio);

	//encoder RIGHT
	encoderRight = new Encoder("encoder right", PORT_NUMBER_ENCODER_RIGHT_A, PORT_NUMBER_ENCODER_RIGHT_B, ticksPerRevolution, wheelDiameter, gearRatio);

	//motor LEFT
	motorLeft = new Motor("motor left", PORT_NUMBER_MOTOR_LEFT_PWM,
	PORT_NUMBER_MOTOR_LEFT_BACKWARD, PORT_NUMBER_MOTOR_LEFT_FORWARD, pwmRange);

	//motor RIGHT
	motorRight = new Motor("motor right", PORT_NUMBER_MOTOR_RIGHT_PWM,
	PORT_NUMBER_MOTOR_RIGHT_BACKWARD, PORT_NUMBER_MOTOR_RIGHT_FORWARD, pwmRange);

	pwmSetMode (PWM_MODE_MS);
	pwmSetRange(pwmRange);
	pwmSetClock(pwmDivisor);

	// pwmFrequency in Hz = 19.2 MHz / pwmClock / pwmRange
	int frequency = 19200000 / pwmDivisor / pwmRange;

	printf("frequency=%d Hz (divisor=%d, range=%d)\n", frequency, pwmDivisor,
			pwmRange);

	//set-up interrupt service routine for detecting the rotation of the motor by the encoder
	wiringPiISR(encoderLeft->getPortA(), INT_EDGE_RISING, &interruptHandlerEncoderLeft);
	wiringPiISR(encoderRight->getPortA(), INT_EDGE_RISING, &interruptHandlerEncoderRight);
}

// ================================================================
// ===             INITIAL PID CONTROLER SETUP                  ===
// ================================================================

void setupPIDControler() {

	Kp = 8.00;
	Ki = 40.00;
	Kd = 0.20;
	Kangle = 0.0001;
	set_point = 0.0; //will be properly set after calibration

	integral_error = 0;
	last_error = 0;
}

void setup() {
	setupSensors();
	setupMotors();
	setupPIDControler();
}

void setValueToMotors(float u) {
	motorLeft->setValue(u);
	motorRight->setValue(u);
}

// ================================================================
// ===                    CALIBRATE		                        ===
// ================================================================

void waitBeforeCalibrate() {
	int secUntilCalibration = 5;
	printf("Starting calibration in %d seconds\n", secUntilCalibration);

	while (secUntilCalibration > 0) {
		//sleep for 1 second
		//usleep(1000000);
		for (int i = 0; i < 1; i++) {
			usleep(1000000);
		}
		secUntilCalibration--;
		printf("Starting calibration in %d seconds\n", secUntilCalibration);
	}
	printf("Calibrating for approximately 5 seconds...\n");
}

void calibrateSetPoint() {
	struct timespec lastTime;
	struct timespec currentTime;

	int countCalibrationAngles = 0;
	double meanCalibrationAngle = 0.0;
	long double sumCalibrationAngle = 0.0;
	float calibrationAngle = 0.0;

	clock_gettime(CLOCK_REALTIME, &lastTime);
	int oldTime = lastTime.tv_sec;

	do {
		mpu.readSensor();

		// display angle in degrees from accelerometer
		double accelerometerAngle = atan2(mpu.ay, mpu.az);

		// display angle in degrees from accelerometer
		printf("accelerometerAngle  %7.2f    ",
				accelerometerAngle * 180 / M_PI);

		printf("calibrationAngle  %7.2f    ", accelerometerAngle * 180 / M_PI);
		sumCalibrationAngle += accelerometerAngle;
		countCalibrationAngles++;
		printf("\n");

		clock_gettime(CLOCK_REALTIME, &currentTime);
	} while (currentTime.tv_sec < (oldTime + 5));

	printf("Calibration completed!\n");
	meanCalibrationAngle = sumCalibrationAngle / countCalibrationAngles;
	printf("meanCalibrationAngle = %f [rad] %f [deg]\n", meanCalibrationAngle,
			meanCalibrationAngle * 180 / M_PI);
	set_point = meanCalibrationAngle;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

	mpu.readSensor();

	// display angle in degrees from accelerometer
	double accelerometerAngle = atan2(mpu.ay, mpu.az);

	double gyroscopeRate = mpu.gsx / 180 * M_PI;

	complementaryFilter.updateValue(gyroscopeRate, accelerometerAngle);
	double angle = complementaryFilter.getAngle();
	double dt = complementaryFilter.dt;

	long encoderLeftCounter = encoderLeft->getCounter();
	long encoderRightCounter = encoderRight->getCounter();
	long encoodersCounterAverage = (encoderLeftCounter + encoderRightCounter)/2;
	float setPointAngleOffset = (float) (Kangle*encoodersCounterAverage);
	cout<<"encoderLeftCounter="<<encoderLeftCounter<<"\t encoderRightCounter="<<encoderRightCounter<<"\t encoodersCounterAverage="<<encoodersCounterAverage<<"\t setPointAngleOffset="<<setPointAngleOffset<<endl;

	//PID
	float error = angle - (set_point - setPointAngleOffset);
	integral_error += error * dt;
	float differential_error = (float) (error - last_error) / dt;
	last_error = error;

	printf("set_point=%f [rad] set_point=%f [deg] setPointAngleOffset=%f error=%f dt=%f integral_error=%f differential_error=%f\n",
			set_point, set_point * 180 / M_PI, setPointAngleOffset, error, dt, integral_error, differential_error);

	float up = Kp * error;
	float ui = Ki * integral_error;
	float ud = Kd * differential_error;
	float u = up + ui + ud;

	printf("up=%f ui=%f ud=%f u=%f\n", up, ui, ud, u);

	setValueToMotors(u);

	printf("countsEncoderLeft = %ld\t stepsEncoderLeft = %ld\t distanceEncoderLeft = %ld [cm]\t countsEncoderRight = %ld\t stepsEncoderRight = %ld\t distanceEncoderRight = %ld [cm]\t", encoderLeft->getCounter(), encoderLeft->getSteps(), encoderLeft->getDistance(), encoderRight->getCounter(), encoderRight->getSteps(), encoderRight->getDistance());

	printf("\n\n");
}

int main() {
	setup();

	usleep(100000);

	waitBeforeCalibrate();
	mpu.calibrateGyroscopes();
	calibrateSetPoint();
	while (true) {
		loop();

	}
	shutdownMotors();

	return 0;
}
