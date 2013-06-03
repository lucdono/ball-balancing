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
 * @file    PID.hpp
 * @brief   PID Controller data structure and functions.
 * @details This file contains the PID data structure type and its update
 *          function declaration.
 *
 * @addtogroup Control
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
/* Defines and Macros.                                                       */
/*===========================================================================*/

/**
 * @brief		Scale an input value within a given range.
 *
 * @param[in]	in		the value to scale.
 * @param[in]	inMin	the lower bound of input value.
 * @param[in]	inMax	the upper bound of input value.
 * @param[in]	outMin	the lower bound of output value.
 * @param[in]	outMax	the upper bound of utput value.
 *
 * @return		the scaled value.
 */
#define map(in, inMin, inMax, outMin, outMax) \
	(((float)in - (float)inMin) / \
	((float)inMax - (float)inMin) * \
	((float)outMax - (float)outMin) + (float)outMin)

/**
 * @brief		Constrain a value within a range.
 *
 * @param[in]	value	the value to constrain.
 * @param[in]	min		lower range limit.
 * @param[in]	max		upper range limit.
 *
 * @return		The constrained value.
 * @retval		value	if the value is within the given range.
 * @retval		min		if the value exceeds lower range bound
 * @retval		max		if the value exceeds upper range bound
 */
#define constrain(value, min, max) \
	( (value<min)? min: ((value>min)?max:value))

/*===========================================================================*/
/* Types definitions.                                                        */
/*===========================================================================*/

/*
 * @brief    PID controller class.
 */
class PID {
private:
	float P; 					//!< Proportional gain.
	float I; 					//!< Integral gain.
	float D; 					//!< Derivative gain.
	float lastPosition; 		//!< Last reference value.
	int previousPIDTime; 		//!< Previous PID update time.
	bool firstPass; 			//!< Initialization flag.
	float integratedError; 		//!< Last integral error.
	float windupGuard; 			//!< Integral saturation guard.

public:
	PID(float P, float I, float D);
	~PID();

	void setP(float val);
	void setI(float val);
	void setD(float val);
	float updatePID(float targetPosition, float currentPosition);

};

#endif /* PID_HPP_ */

/** @} */
