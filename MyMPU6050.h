/*
 * MyMPU6050.h
 *
 *  Created on: 23.08.2015
 *      Author: andy
 */

#include "MPU6050.h"

#ifndef MYMPU6050_H_
#define MYMPU6050_H_

class MyMPU6050 : public MPU6050 {
public:
	MyMPU6050();
	virtual ~MyMPU6050();
	void calibrateGyroscopes();
	void readSensor();
	int16_t ax, ay, az, gx, gy, gz;
	double gsx, gsy, gsz;
private:
	void correct_offset(int16_t gyro_value, int16_t * gyro_offset);
};

#endif /* MYMPU6050_H_ */
