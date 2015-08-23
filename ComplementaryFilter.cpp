/*
 * ComplementaryFilter.cpp
 *
 *  Created on: 23.08.2015
 *      Author: andy
 */

#include "ComplementaryFilter.h"

#include <math.h>
#include <cstdio>

ComplementaryFilter::ComplementaryFilter() {
	// TODO Auto-generated constructor stub

}

ComplementaryFilter::~ComplementaryFilter() {
	// TODO Auto-generated destructor stub
}

void ComplementaryFilter::updateValue(double gyroRate,
		double accelerometerAngle, double dt) {
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
