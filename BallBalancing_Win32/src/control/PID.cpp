/*
 * BallBalancing - Copyright (C) 2013 Luca D'Onofrio.
 *
 * This file is part of BallBalancing Project
 *
 * BallBalancing is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * BallBalancing is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * This firmware is based on OpenCV 2.4.5, see <http://www.opencv.org>.
 */

/**
 * @file    PID.cpp
 * @brief   PID Controller update function code.
 *
 * @addtogroup Control
 * @{
 */

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/

#include <math.h>
#include "PID.hpp"

/*===========================================================================*/
/* External functions.                                                       */
/*===========================================================================*/

/**
 * @brief		The constructor.
 *
 * @param[in]	P	Proportional gain.
 * @param[in]	I	Integral gain.
 * @param[in]	D	Derivative gain.
 */
PID::PID(float P, float I, float D) {
	this->firstPass = true;
	this->I = I;
	this->P = P;
	this->D = D;
	this->previousPIDTime = 0;
	this->lastPosition = 0;
	this->windupGuard = 20;
	this->integratedError = 0;
}

/**
 * @brief 		The destructor.
 */
PID::~PID(void) {

}

/**
 * @brief		Set P value.
 *
 * @param[in]	val	the value to set.
 */
void PID::setP(float val){
	this->P = val;
}

/**
 * @brief		Set D value.
 *
 * @param[in]	val	the value to set.
 */
void PID::setD(float val){
	this->D = val;
}

/**
 * @brief		Set I value.
 *
 * @param[in]	val	the value to set.
 */
void PID::setI(float val){
	this->I = val;
}

/**
 * @brief         	PID controller update function.
 * @details       	Compute the PID output according to the current reference
 *                	signal value and the desired target reference value.
 *
 * @param[in] 		targetPosition   the target reference value.
 * @param[in] 		currentPosition  current signal value.
 *
 * @return    		the actuator input signal needed to reach the target
 * 					reference value.
 */
float PID::updatePID(float targetPosition, float currentPosition) {
	float error;
	float dTerm;
	SYSTEMTIME systemTime;

	/*
	 * Compute elapsed time since last update
	 */
	GetSystemTime (&systemTime);
	long currentTime = systemTime.wMilliseconds;
	float deltaPIDTime = (float) ((currentTime - previousPIDTime)
			/ (float) (1e3));

	this->previousPIDTime = currentTime;

	/*
	 * Compute reference error
	 */
	error = targetPosition - currentPosition;

	/*
	 * Initialize PID
	 */
	if (this->firstPass) {
		this->firstPass = false;
		return (constrain(error, -this->windupGuard, this->windupGuard));
	}

	/*
	 * Compute integral error
	 */
	this->integratedError += (error * deltaPIDTime);
	this->integratedError = constrain(this->integratedError, -this->windupGuard,
			this->windupGuard);

	/*
	 * Compute derivative term
	 */
	dTerm = this->D * (currentPosition - this->lastPosition) / deltaPIDTime;

	this->lastPosition = currentPosition;

	/*
	 * Compute PID output signal
	 */
	return (this->P * error) + (this->I * (this->integratedError)) + dTerm;
}

/** @} */
