/*
 * compute.hpp
 *
 *  Created on: 02/giu/2013
 *      Author: Luca
 */

#ifndef COMPUTE_HPP_
#define COMPUTE_HPP_

#include "Serial.hpp"
#include "PID.hpp"
#include "kalman.hpp"

class Compute {
private:
	//Serial Driver
	Serial *serial;
	PID *pidX;
	PID *pidY;
	Kalman *kalmanX;
	Kalman *kalmanY;

	int width;
	int height;

public:
	//Initialize Serial communication with the given COM port
	Compute(int width, int height);

	//Close the connection
	~Compute();

	//Check if we are actually connected
	bool IsConnected();

	void Update(int x, int y);

};


#endif /* COMPUTE_HPP_ */
