/*
 * kalman.cpp
 *
 *  Created on: 02/giu/2013
 *      Author: Luca
 */

#include "kalman.hpp"

Kalman::Kalman(float Q, float R) {
	this->Q = Q;
	this->R = R;

	this->x_est_last = 0;
	this->P_last = 0;
	this->K = 0;
	this->P = 0;
	this->P_temp = 0;
	this->x_temp_est = 0;
	this->x_est = 0;
}

Kalman::~Kalman() {

}

float Kalman::update(float measure) {
	//do a prediction
	this->x_temp_est = this->x_est_last;
	this->P_temp = this->P_last + this->Q;

	//calculate the Kalman gain
	this->K = this->P_temp * (1.0 / (this->P_temp + this->R));

	//correct
	this->x_est = this->x_temp_est + this->K * (measure - this->x_temp_est);
	//we have our new system

	this->P = (1 - this->K) * this->P_temp;

	//update our last's
	this->P_last = this->P;
	this->x_est_last = this->x_est;

	return(this->x_est);
}
