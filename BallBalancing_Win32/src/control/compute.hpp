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
 * @file    Compute.hpp
 * @brief   Main control algorithm.
 *
 * @addtogroup Control
 * @{
 */

#ifndef COMPUTE_HPP_
#define COMPUTE_HPP_

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/

#include "Serial.hpp"
#include "PID.hpp"
#include "kalman.hpp"

/*===========================================================================*/
/* Defines and Macros                                                        */
/*===========================================================================*/

/**
 * @brief		Max servo angle absolute value
 */
#define MAX_ANGLE	90.0f

/*===========================================================================*/
/* Types definitions.                                                        */
/*===========================================================================*/

/*
 * @brief    Main control class.
 */
class Compute {
private:
	Serial *serial;			//!< The serial driver instance.
	PID *pidX;				//!< PID for X coordinate
	PID *pidY;				//!< PID for Y coordinate
	Kalman *kalmanX;		//!< Kalman filter for X coordinate
	Kalman *kalmanY;		//!< Kalman filter for X coordinate

	int width;				//!< Camera frame width
	int height;				//!< Camera frame height

public:
	Compute(int width, int height);
	~Compute();

	bool IsConnected();
	void Update(int x, int y);

};

#endif /* COMPUTE_HPP_ */

/** @} */
