/*
 * ComplementaryFilter.cpp
 *
 *  Created on: 23.08.2015
 *      Author: andy
 */

#include "ComplementaryFilter.h"

#include <math.h>
#include <cstdio>
#include <time.h>

ComplementaryFilter::ComplementaryFilter() {
}

ComplementaryFilter::~ComplementaryFilter() {
}

void ComplementaryFilter::calculateDt() {
	// calculate dt based on the current time and the previous measurement time
	struct timespec currentTime;
	clock_gettime(CLOCK_REALTIME, &currentTime);
	long dt_nanoseconds = currentTime.tv_nsec - lastTime.tv_nsec;
	long dt_sec = currentTime.tv_sec - lastTime.tv_sec;
	/**
	 * dt in seconds
	 */
	dt = (dt_sec * 1000000000 + dt_nanoseconds) * 1e-9;

	lastTime.tv_nsec = currentTime.tv_nsec;
	lastTime.tv_sec = currentTime.tv_sec;
}

void ComplementaryFilter::updateValue(double gyroRate,
		double accelerometerAngle) {

	calculateDt();

	//complementary filter
	double K = 0.49 / (0.49 + dt);
	double gyroscopeAngle = gyroRate * dt;

	printf("accelerometerAngle %7.2f ; gyroscopeAngle %7.2f ",
			accelerometerAngle * 180 / M_PI, gyroscopeAngle * 180 / M_PI);

	angle = K * (angle + gyroscopeAngle) + (1 - K) * accelerometerAngle;
	printf("K = %.5f filtered angle %.4f [rad] %7.2f [deg]", K, angle,
			angle * 180 / M_PI);

	printf("\n");
}

double ComplementaryFilter::getAngle() {
	return this->angle;
}
