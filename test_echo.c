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

int triggerPort = 6;
int echoPort = 13;

int counter = 0;

struct timespec start_time;

unsigned long int diff_nsec;
unsigned long int distance_mm;

void shutdown(void) {

}

void up() {
	clock_gettime(CLOCK_REALTIME, &start_time);
//	printf("echo 0\n");
}

unsigned long int get_time_diff_nsec(struct timespec time1, struct timespec time2) {
	unsigned long int diff_sec = time2.tv_sec - time2.tv_sec;
	unsigned long int diff_nsec = 1000000000 * diff_sec + time2.tv_nsec - time1.tv_nsec;
	return diff_nsec;
}

void down() {
	struct timespec stop_time;
	clock_gettime(CLOCK_REALTIME, &stop_time);

	diff_nsec = get_time_diff_nsec(start_time, stop_time);

	distance_mm = diff_nsec / 5800;
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

void measure_distance() {
	counter = 0;
	digitalWrite(triggerPort, HIGH);
	struct timespec ONE_MICROSECOND;
	ONE_MICROSECOND.tv_sec = 0;
	ONE_MICROSECOND.tv_nsec = 1000;
	struct timespec remaining;
	nanosleep(&ONE_MICROSECOND, &remaining);
	digitalWrite(triggerPort, LOW);

	while (digitalRead(echoPort) == LOW) {
		// wait
	}
	up();
	while (digitalRead(echoPort) == HIGH) {
		// wait
	}
	down();
}

int main(void) {
	init();

	while (1) {
		struct timespec timer_start;
		clock_gettime(CLOCK_REALTIME, &timer_start);

		measure_distance();

		struct timespec timer_stop;
		clock_gettime(CLOCK_REALTIME, &timer_stop);
		unsigned long int nsec_elapsed = get_time_diff_nsec(timer_start, timer_stop);

		printf("echo distance = %'ld mm ; impulse duration = %'ld nsec ; total_time = %'ld nsec (%'ld usec)\n",
				distance_mm, diff_nsec, nsec_elapsed, nsec_elapsed / 1000);

		delay(1000);
	}

	shutdown();
	return 0;
}
