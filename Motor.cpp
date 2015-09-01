/*
 * Motor.cpp
 *
 *  Created on: Aug 29, 2015
 *      Author: kaladze
 */

#include <iostream>
#include "Motor.h"
#include "wiringPi.h"

Motor::Motor(string name, int pwmPort, int backwardPort, int forwardPort,
		int pwmRange) {
	this->name = name;
	this->pwmPort = pwmPort;
	this->backwardPort = backwardPort;
	this->forwardPort = forwardPort;
	this->backwardValue = 0;
	this->forwardValue = 0;
	this->pwmRange = pwmRange;

	this->mapping_u[0] = 0;
	this->mapping_u[1] = 10;
	this->mapping_u[2] = 100;
	this->mapping_pwm[0] = 0;
	this->mapping_pwm[1] = 13;
	this->mapping_pwm[2] = 100;

	pinMode(this->pwmPort, PWM_OUTPUT);
	pinMode(this->backwardPort, OUTPUT);
	pinMode(this->forwardPort, OUTPUT);

	pwmWrite(this->pwmPort, LOW);
	digitalWrite(this->backwardPort, LOW);
	digitalWrite(this->forwardPort, LOW);

	cout<<"Created new motor: name = "<<this->name<<"\t pwmPort = "<<this->pwmPort<<"\t backwardPort = "<<this->backwardPort<<"\t forwardPort = "<<this->forwardPort<<"\t pwmRange = "<<this->pwmRange<<endl;
}

void Motor::setValue(float target) {
	int localBackwardValue;
	int localForwardValue;

	//limit the target value to the range -1 .. 1
	if (target > 1)
		target = 1;
	if (target < -1)
		target = -1;

	//set direction
	if (target > 0) {
		//forward_value
		localBackwardValue = LOW;
		localForwardValue = HIGH;
	} else if (target < 0) {
		//backward_value
		localBackwardValue = HIGH;
		localForwardValue = LOW;
	} else {
		localBackwardValue = LOW;
		localForwardValue = LOW;
	}

	if (this->backwardValue != localBackwardValue) {
		digitalWrite(this->backwardPort, localBackwardValue);
	}
	if (this->forwardValue != localForwardValue) {
		digitalWrite(this->forwardPort, localForwardValue);
	}

	this->backwardValue = localBackwardValue;
	this->forwardValue = localForwardValue;

	//set power
	if (target < 0)
		target = -target;

	//find interpolation range
	int idx;
	for (idx = 0; idx < 3; idx++) {
		if (mapping_u[idx] > target * 100.0) {
			//found
			break;
		}
	}

	if (idx <= 0)
		idx = 1;
	if (idx > 2) {
		idx = 2;
	}

	// interpolate
	float u1 = mapping_u[idx - 1];
	float u2 = mapping_u[idx];
	float p1 = mapping_pwm[idx - 1];
	float p2 = mapping_pwm[idx];
	float power_percent = p1 + (float) (target * 100.0 - u1) * (float) ((p2 - p1) / (u2 - u1));

	this->setPower(power_percent);

	cout<<this->name<<": idx = "<<idx<<"\t target = "<<target<<"\t power_percent = "<<power_percent<<"\t backwardValue = "<<this->backwardValue<<"\t forwardValue = "<<this->forwardValue<<endl;
}

void Motor::setPower(float powerPercent){
	//scale to pwm_range
	int power = this->pwmRange * (powerPercent / 100);
	pwmWrite(this->pwmPort, power);
	//cout<<"power = "<<power<<endl;
}

Motor::~Motor() {
	pwmWrite(this->pwmPort, 0);
	digitalWrite(this->backwardPort, LOW);
	digitalWrite(this->forwardPort, LOW);
}

