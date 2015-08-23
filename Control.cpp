/*
 * Control.cpp
 *
 *  Created on: Aug 2, 2015
 *      Author: kaladze
 */

#include <stdio.h>
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

int pwm_divisor = 40;
int pwm_range = 1000;

int percentage = 0;

int counterLeft = 0;
int previousCounterLeft = 0;
int stepsLeft;

int counterRight = 0;
int previousCounterRight = 0;
int stepsRight;

float Kp;
float Ki;
float Kd;
float set_point;

float integral_error;
float last_error;

int backward_value;
int forward_value;

float mapping_u[3] = {0, 1, 100};
int mapping_pwm[3] = {0, 13, 100};

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


void shutdown(void) {
	pwmWrite(port_motor_left_pwm, 0);
	digitalWrite(port_motor_left_backward, LOW);
	digitalWrite(port_motor_left_forward, LOW);

	pwmWrite(port_motor_right_pwm, 0);
	digitalWrite(port_motor_right_backward, LOW);
	digitalWrite(port_motor_right_forward, LOW);
}

void interruptHandlerLeft(void) {
	int otherPinValue = digitalRead(port_encoder_left_b);
	if (otherPinValue) {
		counterLeft++;
	} else {
		counterLeft--;
	}
}

void interruptHandlerRight(void) {
	int otherPinValue = digitalRead(port_encoder_right_b);
	if (otherPinValue) {
		counterRight--;
	} else {
		counterRight++;
	}
}

void term(int signum) {
	shutdown();
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
	pinMode(port_encoder_left_a, INPUT);
	pinMode(port_encoder_left_b, INPUT);

	//encoder RIGHT
	pinMode(port_encoder_right_a, INPUT);
	pinMode(port_encoder_right_b, INPUT);

	//motor LEFT
	pinMode(port_motor_left_pwm, PWM_OUTPUT);
	pinMode(port_motor_left_backward, OUTPUT);
	pinMode(port_motor_left_forward, OUTPUT);

	digitalWrite(port_motor_left_backward, LOW);
	digitalWrite(port_motor_left_forward, LOW);

	//motor RIGHT
	pinMode(port_motor_right_pwm, PWM_OUTPUT);
	pinMode(port_motor_right_backward, OUTPUT);
	pinMode(port_motor_right_forward, OUTPUT);

	digitalWrite(port_motor_right_backward, LOW);
	digitalWrite(port_motor_right_forward, LOW);

	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(pwm_range);
	pwmSetClock(pwm_divisor);

	// pwmFrequency in Hz = 19.2 MHz / pwmClock / pwmRange
	int frequency = 19200000 / pwm_divisor / pwm_range;

	printf("frequency=%d Hz (divisor=%d, range=%d)\n", frequency, pwm_divisor,
			pwm_range);

	wiringPiISR(port_encoder_left_a, INT_EDGE_RISING, &interruptHandlerLeft);
	wiringPiISR(port_encoder_right_a, INT_EDGE_RISING, &interruptHandlerRight);
}

// ================================================================
// ===             INITIAL PID CONTROLER SETUP                  ===
// ================================================================

void setupPIDControler() {

	Kp = 8.00;
	Ki = 1.00;
	Kd = 0.20;
	set_point = 0.0;//will be properly set after calibration

	integral_error = 0;
	last_error = 0;

}


void setup(){
	setupSensors();
	setupMotors();
	setupPIDControler();
}

void setValue(float target, float dt){
	int local_backward_value;
	int local_forward_value;

	//limit the target value to the range -1 .. 1
	if (target > 1) target = 1;
	if (target < -1) target = -1;

	//set direction
	if (target > 0){
		//forward_value
		local_backward_value = LOW;
		local_forward_value = HIGH;
	}
	else if (target < 0){
		//backward_value
		local_backward_value = HIGH;
		local_forward_value = LOW;
	}
	else{
		local_backward_value = LOW;
		local_forward_value = LOW;
	}

	if (backward_value != local_backward_value){
		digitalWrite(port_motor_left_backward, local_backward_value);
	}
	if (forward_value != local_forward_value){
		digitalWrite(port_motor_left_forward, local_forward_value);
	}

	if (backward_value != local_backward_value){
		digitalWrite(port_motor_right_backward, local_backward_value);
	}
	if (forward_value != local_forward_value){
		digitalWrite(port_motor_right_forward, local_forward_value);
	}

	backward_value = local_backward_value;
	forward_value = local_forward_value;

	//set power
	if (target < 0) target = -target;

	//find interpolation range
	int idx;
	for(idx=0; idx<3; idx++){
		if(mapping_u[idx] > target*100.0){
			//found
			break;
		}
	}

	if(idx<=0) idx = 1;
	if(idx>2){
		idx = 2;
	}

	// interpolate
	float u1 = mapping_u[idx - 1];
	float u2 = mapping_u[idx];
	float p1 = mapping_pwm[idx - 1];
	float p2 = mapping_pwm[idx];
	float power_percent = p1 + (float) (target * 100.0 - u1) * (float) ((p2 - p1) / (u2 - u1));

	printf("idx= %d: target=%f power_percent=%f\n", idx, target, power_percent);

	//scale to pwm_range
	int power = pwm_range * (power_percent / 100);
	pwmWrite(port_motor_left_pwm, power);
	pwmWrite(port_motor_right_pwm, power);
}


// ================================================================
// ===                    CALIBRATE		                        ===
// ================================================================

void waitBeforeCalibrate() {
	int secUntilCalibration = 5;
	printf("Starting calibration in %d seconds\n", secUntilCalibration);

	while(secUntilCalibration > 0){
		//sleep for 1 second
		//usleep(1000000);
		for(int i=0; i<1; i++){ usleep(1000000);}
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

	//PID
	float error = angle - set_point;
	integral_error += error * dt;
	float differential_error = (float) (error - last_error) / dt;
	last_error = error;

	printf("angle=%f error=%f dt=%f integral_error=%f differential_error=%f\n",
			angle, error, dt, integral_error, differential_error);

	float up = Kp * error;
	float ui = Ki * integral_error;
	float ud = Kd * differential_error;
	float u = up + ui + ud;

	printf("up=%f ui=%f ud=%f u=%f\n", up, ui, ud, u);

	setValue(u, dt);

	stepsLeft = counterLeft - previousCounterLeft;
	previousCounterLeft = counterLeft;

	stepsRight = counterRight - previousCounterRight;
	previousCounterRight = counterRight;

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
	shutdown();

	return 0;
}
