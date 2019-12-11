/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */
#include "Arduino.h"
#include "RBEPID.h"

//Class constructor
RBEPID::RBEPID() {

}

//Function to set PID gain values
void RBEPID::setpid(float P, float I, float D) {
	kp = P;
	ki = I;
	kd = D;
}

/**
 * calc the PID control signel
 *
 * @param setPoint is the setpoint of the PID system
 * @param curPosition the current position of the plan
 * @return a value from -1.0 to 1.0 representing the PID control signel
 */
float RBEPID::calc(double setPoint, double curPosition) {
	if((curPosition - setPoint < 0 && err_positive) || (curPosition - setPoint > 0 && !err_positive)) {
			clearIntegralBuffer();
			err_positive = !err_positive;
	}
	// calculate error
	float err = setPoint - curPosition;
	// calculate derivative of error
	float err_d = last_error - err;
	// calculate integral error. Running average is best but hard to implement
	sum_error = sum_error + err - errors[err_index];

	float err_i = sum_error / err_size;
	// sum up the error value to send to the motor based off gain values.
	//TODO

	float out = err * kp + err_d * kd + err_i * ki;	// simple P controller
	//return the control signal from -1 to 1
	if (out > 1)
		out = 1;
	if (out < -1)
		out = -1;

	errors[err_index] = err;
	updateErrorIndex();
	last_error = err;
	return out;
}

/**
 * Clear the internal representation fo the integral term.
 *
 */
void RBEPID::clearIntegralBuffer() {
	sum_error = 0;
}

void RBEPID::updateErrorIndex() {
	err_index ++;
	if(err_index == err_size)
		err_index = 0;
}
