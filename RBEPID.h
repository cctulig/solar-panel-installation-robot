/*
 * Messages.h
 *
 *  Created on: 10/1/16
 *      Author: joest
 */

#ifndef PID_H_
#define PID_H_


class RBEPID {

public:
	RBEPID();
	float kp = 0.00015;
	float ki = 0;
	float kd = 0;
	float last_error = 0;
	float sum_error = 0;
	int err_index = 0;
	float errors[16] =  { 0 };
	int err_size = 16;
	int sampleRateMs = 5;
	bool err_positive = true;
	/**
	 * setpid set PID constants
	 */
	void setpid(float P, float I, float D);
	/**
	 * calc the PID control signel
	 *
	 * @param setPoint is the setpoint of the PID system
	 * @param curPosition the current position of the plan
	 * @return a value from -1.0 to 1.0 representing the PID control signel
	 */
	float calc(double setPoint, double curPosition);
	/**
	 * Clear the internal representation fo the integral term.
	 *
	 */
	void clearIntegralBuffer();

	void updateErrorIndex();
private:

};

#endif
