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
 * @file    PID.h
 * @brief   PID Controller data structure and functions.
 * @details This file contains the PID data structure type and its update
 *          function declaration.
 *
 * @addtogroup Utils
 * @{
 */

#ifndef PID_HPP_
#define PID_HPP_

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/
#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

/*===========================================================================*/
/* Types definitions.                                                        */
/*===========================================================================*/

#define map(in, inMin, inMax, outMin, outMax) \
	(((double)in - (double)inMin) / ((double)inMax - (double)inMin) * ((double)outMax - (double)outMin) + (double)outMin)

#define constrain(value, min, max) \
	( (value<min)? min: ((value>min)?max:value))

/*
 * @brief    PID controller data structure.
 */
class PID {
private:
	float P; /**< Proportional gain */
	float I; /**< Integral gain */
	float D; /**< Derivative gain */
	float lastPosition; /**< Last reference value */
	int previousPIDTime; /**< Previous PID update time */
	bool firstPass; /**< Initialization flag */
	float integratedError; /**< Last integral error */
	float windupGuard; /**< Integral saturation guard */

public:
	PID(float P, float I, float D);
	~PID();
	float updatePID(float targetPosition, float currentPosition);

};

#endif /* PID_HPP_ */

/** @} */
