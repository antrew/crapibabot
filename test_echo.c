/*
 * test_ir.c
 *
 *  Created on: 05.01.2016
 *      Author: Andrey Vetlugin <antrew@gmail.com>
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "wiringPi.h"
//#include "ports.h"

int triggerPort=6;
int echoPort=13;

int counter=0;

struct timespec start_time;

void shutdown(void) {
	
}

void up() {
	clock_gettime(CLOCK_REALTIME, &start_time);
//	printf("echo 0\n");
}

void down() {
	struct timespec stop_time;
	clock_gettime(CLOCK_REALTIME, &stop_time);
	unsigned long int diff_sec = stop_time.tv_sec - start_time.tv_sec;
	unsigned long int diff_nsec = 1000000000 * diff_sec + stop_time.tv_nsec - start_time.tv_nsec;
	unsigned long int distance_mm = diff_nsec / 5800;
	printf("echo 1 %ld %ld\n", diff_nsec, distance_mm);
}

void interruptHandlerEcho(void) {
	if (counter == 0) {
		// rising
		up();
		counter++;
	} else {
		// falling
		down();
	}
}

void term(int signum) {
	shutdown();
	exit(1);
}

void init(void) {
	printf("A");
	wiringPiSetupGpio();
	printf("B");

	// SIGTERM handler
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = term;
	sigaction(SIGTERM, &action, NULL);
	printf("C\n");

	pinMode(triggerPort, OUTPUT);

	pinMode(echoPort, INPUT);
	pullUpDnControl(echoPort, PUD_OFF);
//	int res = wiringPiISR(echoPort, INT_EDGE_BOTH, &interruptHandlerEcho);
//	printf("%d\n", res);
}

int main(void) {
	init();

	while (1) {

		counter=0;
		digitalWrite(triggerPort, HIGH);
		delay(1);
		digitalWrite(triggerPort, LOW);

		while ( digitalRead(echoPort) == LOW ) {
			// wait
		}
		up();
		while ( digitalRead(echoPort) == HIGH ) {
			// wait
		}
		down();

		delay(1000);
	}

	shutdown();
	return 0;
}
