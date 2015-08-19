/*
 * Control.cpp
 *
 *  Created on: Aug 2, 2015
 *      Author: kaladze
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <signal.h>
#include <time.h>
#include <inttypes.h>
#include "I2Cdev.h"
#include "wiringPi.h"
#include "ports.h"
#include "MPU6050_6Axis_MotionApps20.h"

int pwm_divisor = 40;
int pwm_range = 1000;

int percentage = 0;

int counterLeft = 0;
int previousCounterLeft = 0;
int stepsLeft;

int counterRight = 0;
int previousCounterRight = 0;
int stepsRight;

struct timespec lastTime;
struct timespec currentTime;

float angle;
float K;

float Kp;
float Ki;
float Kd;
float set_point;

float integral_error;
float last_error;

int backward_value;
int forward_value;

float mapping_u[3] = {0, 2.5, 100};
int mapping_pwm[3] = {0, 48, 100};

int countCalibrationAngles = 0;
float meanCalibrationAngle = 0.0;
long double sumCalibrationAngle = 0.0;
float calibrationAngle = 0.0;
int calibrationCompleted = 0;
int calibrationStartTime = 0;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


void shutdown(void) {
	pwmWrite(port_motor_left_pwm, 0);
	digitalWrite(port_motor_left_backward, LOW);
	digitalWrite(port_motor_left_forward, LOW);

	pwmWrite(port_motor_right_pwm, 0);
	digitalWrite(port_motor_right_backward, LOW);
	digitalWrite(port_motor_right_forward, LOW);
}

void interruptHandlerLeft(void) {
	int otherPinValue = digitalRead(port_encoder_left_b);
	if (otherPinValue) {
		counterLeft++;
	} else {
		counterLeft--;
	}
}

void interruptHandlerRight(void) {
	int otherPinValue = digitalRead(port_encoder_right_b);
	if (otherPinValue) {
		counterRight--;
	} else {
		counterRight++;
	}
}

void term(int signum) {
	shutdown();
	exit(1);
}

// ================================================================
// ===         INITIAL SENSORS (acc & gyro) SETUP               ===
// ================================================================

void setupSensors() {
    // initialize device
    printf("Initializing I2C devices...\n");
    mpu.initialize();

    // verify connection
    printf("Testing device connections...\n");
    printf(mpu.testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");

    // load and configure the DMP
    printf("Initializing DMP...\n");
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        printf("Enabling DMP...\n");
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        printf("DMP ready!\n");
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)\n", devStatus);
    }
}

// ================================================================
// ===                 INITIAL MOTORS SETUP                     ===
// ================================================================

void setupMotors() {
	wiringPiSetupGpio();

	// SIGTERM handler
	struct sigaction action;
	memset(&action, 0, sizeof(struct sigaction));
	action.sa_handler = term;
	sigaction(SIGINT, &action, NULL);

	//encoder LEFT
	pinMode(port_encoder_left_a, INPUT);
	pinMode(port_encoder_left_b, INPUT);

	//encoder RIGHT
	pinMode(port_encoder_right_a, INPUT);
	pinMode(port_encoder_right_b, INPUT);

	//motor LEFT
	pinMode(port_motor_left_pwm, PWM_OUTPUT);
	pinMode(port_motor_left_backward, OUTPUT);
	pinMode(port_motor_left_forward, OUTPUT);

	digitalWrite(port_motor_left_backward, LOW);
	digitalWrite(port_motor_left_forward, LOW);

	//motor RIGHT
	pinMode(port_motor_right_pwm, PWM_OUTPUT);
	pinMode(port_motor_right_backward, OUTPUT);
	pinMode(port_motor_right_forward, OUTPUT);

	digitalWrite(port_motor_right_backward, LOW);
	digitalWrite(port_motor_right_forward, LOW);

	pwmSetMode(PWM_MODE_MS);
	pwmSetRange(pwm_range);
	pwmSetClock(pwm_divisor);

	// pwmFrequency in Hz = 19.2 MHz / pwmClock / pwmRange
	int frequency = 19200000 / pwm_divisor / pwm_range;

	printf("frequency=%d Hz (divisor=%d, range=%d)\n", frequency, pwm_divisor,
			pwm_range);

	wiringPiISR(port_encoder_left_a, INT_EDGE_RISING, &interruptHandlerLeft);
	wiringPiISR(port_encoder_right_a, INT_EDGE_RISING, &interruptHandlerRight);
}

// ================================================================
// ===             INITIAL PID CONTROLER SETUP                  ===
// ================================================================

void setupPIDControler() {
	angle = 0;
	K = 0.98;

	Kp = 0.16;
	Ki = 0.04;
	Kd = 0.008;
	set_point = 0.0;//will be properly set after calibration

	integral_error = 0;
	last_error = 0;

	clock_gettime(CLOCK_REALTIME, &lastTime);
}


void setup(){
	setupSensors();
	setupMotors();
	setupPIDControler();
}

void setValue(float target, float dt){
	int local_backward_value;
	int local_forward_value;

	//limit the target value to the range -1 .. 1
	if (target > 1) target = 1;
	if (target < -1) target = -1;

	//set direction
	if (target > 0){
		//forward_value
		local_backward_value = LOW;
		local_forward_value = HIGH;
	}
	else if (target < 0){
		//backward_value
		local_backward_value = HIGH;
		local_forward_value = LOW;
	}
	else{
		local_backward_value = LOW;
		local_forward_value = LOW;
	}

	if (backward_value != local_backward_value){
		digitalWrite(port_motor_left_backward, local_backward_value);
	}
	if (forward_value != local_forward_value){
		digitalWrite(port_motor_left_forward, local_forward_value);
	}

	if (backward_value != local_backward_value){
		digitalWrite(port_motor_right_backward, local_backward_value);
	}
	if (forward_value != local_forward_value){
		digitalWrite(port_motor_right_forward, local_forward_value);
	}

	backward_value = local_backward_value;
	forward_value = local_forward_value;

	//set power
	if (target < 0) target = -target;

	//find interpolation range
	int idx;
	for(idx=0; idx<3; idx++){
		if(mapping_u[idx] > target*100.0){
			//found
			break;
		}
	}

	if(idx<=0) idx = 1;
	if(idx>2){
		idx = 2;
	}

	// interpolate
	float u1 = mapping_u[idx - 1];
	float u2 = mapping_u[idx];
	float p1 = mapping_pwm[idx - 1];
	float p2 = mapping_pwm[idx];
	float power_percent = p1 + (float) (target * 100.0 - u1) * (float) ((p2 - p1) / (u2 - u1));

	printf("idx= %d: target=%f power_percent=%f\n", idx, target, power_percent);

	//scale to pwm_range
	int power = pwm_range * (power_percent / 100);
	pwmWrite(port_motor_left_pwm, power);
	pwmWrite(port_motor_right_pwm, power);
}


// ================================================================
// ===                    CALIBRATE		                        ===
// ================================================================

void calibrate(){
	int secUntilCalibration = 5;
	printf("Starting calibration in %d seconds\n", secUntilCalibration);

	while(secUntilCalibration > 0){
		//sleep for 1 second
		//usleep(1000000);
		for(int i=0; i<1; i++){ usleep(1000000);}
		secUntilCalibration--;
		printf("Starting calibration in %d seconds\n", secUntilCalibration);
	}
	printf("Calibrating for approximately 5 seconds...\n");

	int countCalibrationAngles = 0;
	float meanCalibrationAngle = 0.0;
	long double sumCalibrationAngle = 0.0;
	float calibrationAngle = 0.0;

	clock_gettime(CLOCK_REALTIME, &lastTime);
	int oldTime = lastTime.tv_sec;

	do{
		// if programming failed, don't try to do anything
		if (!dmpReady) return;
		// get current FIFO count
		fifoCount = mpu.getFIFOCount();

		if (fifoCount == 1024) {
			// reset so we can continue cleanly
			mpu.resetFIFO();
			printf("FIFO overflow!\n");
			clock_gettime(CLOCK_REALTIME, &currentTime);

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
		} else if (fifoCount >= 42) {
			// read a packet from FIFO
			mpu.getFIFOBytes(fifoBuffer, packetSize);

			// display quaternion values in easy matrix form: w x y z
			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			printf("areal %6d %6d %6d    ", aaReal.x, aaReal.y, aaReal.z);

			// display Euler angles in degrees
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
			printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);

			// display angle in degrees from accelerometer
			float accelerometerAngle = atan2(aaReal.y, aaReal.z);
			printf("accelerometerAngle  %7.2f    ", accelerometerAngle * 180/M_PI);

			// calculate dt based on the current time and the previous measurement time
			clock_gettime(CLOCK_REALTIME, &currentTime);
			long ms = round ((currentTime.tv_nsec - lastTime.tv_nsec) / 1.0e6); //convert nanoseconds to milliseconds
			int sec = currentTime.tv_sec - lastTime.tv_sec;
			float dt = (float)(sec * 1000 + ms) / 1000;

			printf("currentTimeSEC=%d currentTimeMS=%ld lastTimeSEC=%d lastTimeMS=%ld SEC=%d MS=%ld dt=%f\n", currentTime.tv_sec, currentTime.tv_nsec, lastTime.tv_sec, lastTime.tv_nsec, sec, ms, dt);

			lastTime.tv_nsec = currentTime.tv_nsec;
			lastTime.tv_sec = currentTime.tv_sec;

			//complementary filter
			K = (float) 0.49/(0.49 + dt);
			calibrationAngle = K * (calibrationAngle + ypr[2] * dt) + (1 - K) * accelerometerAngle;

			printf("calibrationAngle  %7.2f    ", calibrationAngle * 180/M_PI);
			sumCalibrationAngle += calibrationAngle;
			countCalibrationAngles++;
		}

		clock_gettime(CLOCK_REALTIME, &currentTime);
	}while(currentTime.tv_sec < (oldTime + 5));

	printf("Calibration completed!\n");
	meanCalibrationAngle = sumCalibrationAngle / countCalibrationAngles;
	printf("meanCalibrationAngle = %f [rad] %f [deg]\n", meanCalibrationAngle, meanCalibrationAngle * 180/M_PI);
	set_point = meanCalibrationAngle;
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop(){
	 // if programming failed, don't try to do anything
	if (!dmpReady) return;
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	if (fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		printf("FIFO overflow!\n");

	// otherwise, check for DMP data ready interrupt (this should happen frequently)
	} else if (fifoCount >= 42) {
		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// display quaternion values in easy matrix form: w x y z
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		printf("realAccelAngle %6d %6d %6d    ", aaReal.x,aaReal.y,aaReal.z);

		// display Euler angles in degrees
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		printf("ypr  %7.2f %7.2f %7.2f    ", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);

		// display angle in degrees from accelerometer
		float accelerometerAngle = atan2(aaReal.y, aaReal.z);
		printf("accelerometerAngle  %7.2f    ", accelerometerAngle * 180/M_PI);

		// calculate dt based on the current time and the previous measurement time
		clock_gettime(CLOCK_REALTIME, &currentTime);
		long ms = round ((currentTime.tv_nsec - lastTime.tv_nsec) / 1.0e6); //convert nanoseconds to milliseconds
		int sec = currentTime.tv_sec - lastTime.tv_sec;
		float dt = (float)(sec * 1000 + ms) / 1000;

        printf("currentTimeSEC=%d currentTimeMS=%ld lastTimeSEC=%d lastTimeMS=%ld SEC=%d MS=%ld dt=%f\n", currentTime.tv_sec, currentTime.tv_nsec, lastTime.tv_sec, lastTime.tv_nsec, sec, ms, dt);

        lastTime.tv_nsec = currentTime.tv_nsec;
        lastTime.tv_sec = currentTime.tv_sec;

        //complementary filter
        K = (float) 0.49/(0.49 + dt);
        angle = K * (angle + ypr[2] * dt) + (1 - K) * accelerometerAngle;
        printf("K = %f filtered angle %f [rad]	 %7.2f [deg]    ", K, angle, angle * 180/M_PI);

		printf("\n");

		//PID
		float error = angle - set_point;
		integral_error += error * dt;
		float differential_error = (float) (error - last_error) / dt;
		last_error = error;

		printf("angle=%f error=%f dt=%f integral_error=%f differential_error=%f\n", angle, error, dt, integral_error, differential_error);

		float up = Kp * error;
		float ui = Ki * integral_error;
		float ud = Kd * differential_error;
		float u = up + ui + ud;

		printf("up=%f ui=%f ud=%f u=%f\n", up, ui, ud, u);

		setValue(u, dt);

		stepsLeft = counterLeft - previousCounterLeft;
		previousCounterLeft = counterLeft;

		stepsRight = counterRight - previousCounterRight;
		previousCounterRight = counterRight;

		printf("\n\n");
	}
}

int main() {
	setup();

	usleep(100000);

	calibrate();
	while (true) {
		loop();


	}
	shutdown();

	return 0;
}
