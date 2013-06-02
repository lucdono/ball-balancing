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
 * @file    Kalman.hpp
 * @brief   Simple Kalman filter.
 *
 * @addtogroup Control
 * @{
 */

#ifndef KALMAN_HPP_
#define KALMAN_HPP_

/*===========================================================================*/
/* Types definitions.                                                        */
/*===========================================================================*/

/*
 * @brief    Kalman filter class.
 */
class Kalman{
private:
	float x_est_last;	//!< Last output estimation.
	float P_last;		//!< Last prediction.
	float Q;			//!< Noise covariance.
	float R;			//!< Noise covariance.
	float K;			//!< Kalman gain.
	float P;			//!< Prediction.
	float P_temp;		//!< System inner state temp value.
	float x_temp_est;	//!< Output estimation temp value.
	float x_est;		//!< Output estimation.

public:
	Kalman(float Q, float R);
	~Kalman();

	float update(float measure);
};


#endif /* KALMAN_HPP_ */

/** @} */
