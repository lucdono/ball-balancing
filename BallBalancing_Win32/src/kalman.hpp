/*
 * kalman.hpp
 *
 *  Created on: 02/giu/2013
 *      Author: Luca
 */

#ifndef KALMAN_HPP_
#define KALMAN_HPP_

class Kalman{
private:
	//initial values for the kalman filter
	float x_est_last;
	float P_last;
	float Q;
	float R;
	float K;
	float P;
	float P_temp;
	float x_temp_est;
	float x_est;

public:
	Kalman(float Q, float R);
	~Kalman();

	float update(float measure);
};


#endif /* KALMAN_HPP_ */
