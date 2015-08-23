/*
 * MyMPU6050.cpp
 *
 *  Created on: 23.08.2015
 *      Author: andy
 */
#include <stdint.h>
#include <stdio.h>
#include "MyMPU6050.h"

#define G_RATE 1.0 / (131)

MyMPU6050::MyMPU6050() {
	// TODO Auto-generated constructor stub

}

MyMPU6050::~MyMPU6050() {
	// TODO Auto-generated destructor stub
}

void MyMPU6050::correct_offset(int16_t gyro_value, int16_t * gyro_offset) {
	if (gyro_value > 0) {
		*gyro_offset -= 1;
	} else if (gyro_value < 0) {
		*gyro_offset += 1;
	} else {
	}
}

void MyMPU6050::calibrateGyroscopes() {
	// calibration of the gyroscope
	int16_t ax, ay, az, gx, gy, gz;

	int16_t gx_offset = this->getXGyroOffsetUser();
	int16_t gy_offset = this->getYGyroOffsetUser();
	int16_t gz_offset = this->getZGyroOffsetUser();

	this->setXGyroOffsetUser(gx_offset);

	for (int i = 0; i < 300; i++) {
		this->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
		printf("%5d %5d ; %5d %5d ; %5d %5d\n", gx, gx_offset, gy, gy_offset,
				gz, gz_offset);

		correct_offset(gx, &gx_offset);
		correct_offset(gy, &gy_offset);
		correct_offset(gz, &gz_offset);

		this->setXGyroOffsetUser(gx_offset);
		this->setYGyroOffsetUser(gy_offset);
		this->setZGyroOffsetUser(gz_offset);
	}
}

void MyMPU6050::readSensor() {

	this->getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	gsx = gx * G_RATE;
	gsy = gy * G_RATE;
	gsz = gz * G_RATE;

//	printf("accelerometer: %5d, %5d, %5d ; %5d, %5d, %5d\n", ax, ay, az);
//	printf("gyroscope:     %5d, %5d, %5d ; %5f, %5f, %5f\n", gx, gy, gz, gsx,
//			gsy, gsz);

}
