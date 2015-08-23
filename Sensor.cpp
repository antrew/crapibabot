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

struct timespec lastTime;
struct timespec currentTime;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MyMPU6050 mpu;
ComplementaryFilter complementaryFilter;

void shutdown(void) {
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

void setup() {
	setupSensors();

	mpu.calibrateGyroscopes();
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

	mpu.readSensor();

	// display angle in degrees from accelerometer
	double accelerometerAngle = atan2(mpu.ay, mpu.az);

	// calculate dt based on the current time and the previous measurement time
	clock_gettime(CLOCK_REALTIME, &currentTime);
	long dt_nanoseconds = currentTime.tv_nsec - lastTime.tv_nsec;
	long dt_sec = currentTime.tv_sec - lastTime.tv_sec;
	/**
	 * dt in seconds
	 */
	double dt = (dt_sec * 1000000000 + dt_nanoseconds) * 1e-9;

	lastTime.tv_nsec = currentTime.tv_nsec;
	lastTime.tv_sec = currentTime.tv_sec;

	double gyroscopeRate = mpu.gsx / 180 * M_PI;

	complementaryFilter.updateValue(gyroscopeRate, accelerometerAngle, dt);
	double angle = complementaryFilter.getAngle();
	printf("filtered angle: %f\n", angle);
}

int main() {
	setup();

	usleep(100000);

	while (true) {
		loop();
	}
	shutdown();

	return 0;
}
