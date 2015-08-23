/*
 * ComplementaryFilter.h
 *
 *  Created on: 23.08.2015
 *      Author: andy
 */

#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

class ComplementaryFilter {
public:
	ComplementaryFilter();
	virtual ~ComplementaryFilter();
	void updateValue(double gyroRate, double accelerometerAngle, double dt);
	double getAngle();
private:
	double angle;
};

#endif /* COMPLEMENTARYFILTER_H_ */
