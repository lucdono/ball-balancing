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
 * @file    Compute.cpp
 * @brief   Main control algorithm.
 *
 * @addtogroup Control
 * @{
 */

#include "serial.hpp"
#include "compute.hpp"
#include "stdint.h"

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief		The constructor.
 *
 * @param[in]	width	Camera frame width.
 * @param[in]	height	Camera frame height.
 */
Compute::Compute(int width, int height) {
	this->serial = new Serial("\\\\.\\COM19");

	this->pidX = new PID(0.08f, 0.0f, -0.03f);
	this->pidY = new PID(0.06f, 0.0f, -0.05f);
	this->kalmanX = new Kalman(0.012f, 0.215f);
	this->kalmanY = new Kalman(0.012f, 0.215f);
	this->width = width;
	this->height = height;
	this->setPoint_x = 0;
	this->setPoint_y = 0;
}

/**
 * @brief		The destructor
 */
Compute::~Compute() {

}

/**
 * @brief		Return PID x axis.
 * @return		The PID instance.
 */
PID *Compute::getPIDX(void) {
	return this->pidX;
}

/**
 * @brief		Return PID y axis.
 * @return		The PID instance.
 */
PID *Compute::getPIDY(void) {
	return this->pidY;
}

/**
 * @brief		Update the x,y coordinates of the set point.
 *
 * @param[in]	x	x axis coordinate of the set point.
 * @param[in]	y	y axis coordinate of the set point.
 */
void Compute::setPoint(int x, int y) {
	this->setPoint_x = x;
	this->setPoint_y = y;
}

/**
 * @brief 		Reset the set point to the center.
 */
void Compute::center() {
	this->setPoint_x = this->width / 2;
	this->setPoint_y = this->height / 2;
}

/**
 * @brief		Check serial connection.
 * @return		TRUE if serial port has been connected, FALSE otherwise.
 */
bool Compute::IsConnected() {
	return this->serial->IsConnected();
}

/**
 * @brief		Main control function.
 * @details		This function is responsible for the control of the plate:
 * 				it makes an estimation of the real position of the object,
 * 				then generate control signals (i.e. angles for each servos)
 * 				using PIDs.
 *
 * @param[in]	x	object x coordinate.
 * @param[in]	y	object y coordinate.
 */
void Compute::Update(int x, int y) {
	char data[2];

	if (serial->IsConnected()) {
		/*
		 * Estimate x and y coordinates
		 */
		float x_est = this->kalmanX->update((float) x);
		float y_est = this->kalmanY->update((float) y);

		/*
		 * Update PIDs
		 */
		float ctrl_x = this->pidX->updatePID(this->setPoint_x, x_est);
		float ctrl_y = this->pidY->updatePID(this->setPoint_y, y_est);

		/*
		 * Scale PIDs output to motor angles
		 */
		data[0] =
				map(ctrl_x,MAX_ANGLE,-MAX_ANGLE,-(float)(this->width / 2),(float)(this->width / 2));
		data[1] =
				map(ctrl_y,MAX_ANGLE,-MAX_ANGLE,-(float)(this->height / 2),(float)(this->height / 2));

		/*
		 * Send data over serial port
		 */
		serial->WriteData(data, 2);
	} else
		printf("Serial port unavailable!\n");
}

/** @} */
