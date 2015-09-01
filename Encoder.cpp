/*
 * Encoder.cpp
 *
 *  Created on: Aug 29, 2015
 *      Author: kaladze
 */

#include "Encoder.h"
#include "wiringPi.h"

Encoder::Encoder(string name, int portA, int portB, int ticksPerRevolution, float wheelDiameter, float gearRatio) {
	this->name = name;
	this->portA = portA;
	this->portB = portB;
	this->counter = 0;
	this->previousCounter = 0;
	this->ticksPerRevolution = ticksPerRevolution;
	this->wheelDiameter = wheelDiameter;
	this->gearRatio = gearRatio;

	//set both ports as input
	pinMode(this->portA, INPUT);
	pinMode(this->portB, INPUT);
}

void Encoder::updateCounters(int pinValue) {
	if (pinValue) {
		this->counter--;
	} else {
		this->counter++;
	}
}

int Encoder::getPortA(){
	return this->portA;
}

int Encoder::getPortB(){
	return this->portB;
}

long Encoder::getSteps(){
	long steps = this->counter - this->previousCounter;
	this->previousCounter = this->counter;
	return steps;
}

long Encoder::getCounter(){
	return this->counter;
}

long Encoder::getDistance(){
	long distance = 0;
	distance = (long) (this->counter * (this->wheelDiameter * M_PI) / (this->ticksPerRevolution * this->gearRatio));
	return distance;
}

void Encoder::resetCounters(){
	this->counter = 0;
	this->previousCounter = 0;
}

Encoder::~Encoder() {
}

