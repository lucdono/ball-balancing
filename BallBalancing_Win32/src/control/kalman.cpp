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
 * @file    Kalman.cpp
 * @brief   Simple Kalman filter code.
 *
 * @addtogroup Control
 * @{
 */

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/

#include "kalman.hpp"

/*===========================================================================*/
/* External functions.                                                       */
/*===========================================================================*/

/**
 * @brief		The constructor.
 *
 * @param[in]	Q	Estimation of the noise covariance.
 * @param[in]	R	Estimation of the noise covariance.
 */
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

/**
 * @brief 		The destructor.
 */
Kalman::~Kalman() {

}

/**
 * @brief		Update estimated value.
 *
 * @param[in]	measure		The measured value
 *
 * @return		The estimated value.
 */
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


/** @} */
