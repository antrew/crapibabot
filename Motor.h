/*
 * Motor.h
 *
 *  Created on: Aug 29, 2015
 *      Author: kaladze
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <string>

using namespace std;

class Motor {
public:
	Motor(string name, int pwmPort, int backwardPort, int forwardPort, int pwmRange);
	void setValue(float target);
	virtual ~Motor();
private:
	void setPower(float powerPercent);
	string name;
	int pwmPort;
	int backwardPort;
	int forwardPort;
	int pwmRange;
	int backwardValue;
	int forwardValue;
	float mapping_u[3];
	int mapping_pwm[3];
};

#endif /* MOTOR_H_ */
