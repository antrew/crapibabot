/*
 * ComplementaryFilter.h
 *
 *  Created on: 23.08.2015
 *      Author: andy
 */

#include <time.h>

#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

class ComplementaryFilter {
public:
	ComplementaryFilter();
	virtual ~ComplementaryFilter();
	void updateValue(double gyroRate, double accelerometerAngle);
	double getAngle();
	double dt;
private:
	double angle;
	void calculateDt();
	struct timespec lastTime;
};

#endif /* COMPLEMENTARYFILTER_H_ */
