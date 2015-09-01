/*
 * Encoder.h
 *
 *  Created on: Aug 29, 2015
 *      Author: kaladze
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <string>
#include <math.h>

using namespace std;

class Encoder {
public:
	Encoder(string name, int portA, int portB, int ticksPerRevolution, float wheelDiameter, float gearRatio);
	int getPortA();
	int getPortB();
	long getSteps();
	long getCounter();
	long getDistance();
	void updateCounters(int pinValue);
	void resetCounters();
	virtual ~Encoder();
private:
	string name;
	int portA;
	int portB;
	long counter;
	long previousCounter;
	int ticksPerRevolution;
	float wheelDiameter;
	float gearRatio;
};

#endif /* ENCODER_H_ */
