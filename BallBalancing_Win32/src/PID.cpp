/*
 XBlade-Copter - Copyright (C) 2012 Luca D'Onofrio.

 This file is part of XBlade-Copter Project

 XBlade-Copter is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 3 of the License, or
 (at your option) any later version.

 XBlade-Copter is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file    PID.c
 * @brief   PID Controller update function code.
 *
 * @addtogroup Utils
 * @{
 */

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/
#include <math.h>
#include "PID.hpp"

PID::PID(float P, float I, float D) {
	this->firstPass = TRUE;
	this->I = I;
	this->P = P;
	this->D = D;
	this->previousPIDTime = 0;
	this->lastPosition = 0;
	this->windupGuard = 20;
	this->integratedError = 0;
}

PID::~PID(void) {

}

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/
/**
 * @brief         PID controller update function.
 * @details       Compute the PID output according to the current reference
 *                signal value and the desired target reference value.
 *
 * @param[in] targetPosition   the target reference value.
 * @param[in] currentPosition  current signal value.
 * @param[in] PIDparamenters   the PID controller parameters.
 *
 * @return    the actuator input signal needed to reach the target reference
 *            value.
 *
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
		this->firstPass = FALSE;
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
