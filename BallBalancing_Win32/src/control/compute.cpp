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

	this->pidX = new PID(0.09f, 0.0f, -0.03f);
	this->pidY = new PID(0.07f, 0.0f, -0.05f);
	this->kalmanX = new Kalman(0.022f, 0.215f);
	this->kalmanY = new Kalman(0.022f, 0.215f);
	this->width = width;
	this->height = height;
	this->setPoint_x = 0;
	this->setPoint_y = 0;
	this->image = NULL;
	this->window = NULL;
	this->sample = 0;
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
		this->Plot(x_est, y_est);
	} else
		printf("Serial port unavailable!\n");
}

void Compute::SetPlotter(char *win, IplImage* image) {
	this->image = image;
	this->window = win;
}

void Compute::Plot(float x_est, float y_est) {
	int i_x_est =
			map(x_est,0.0f,(float)this->width,0.0f,(float)(this->image->height/2));
	int i_x_set =
			map(this->setPoint_x,0.0f,(float)this->width,0.0f,(float)(this->image->height/2));
	int i_y_est =
			map(y_est,0.0f,(float)this->height,(float)(this->image->height/2),(float)(this->image->height));
	int i_y_set =
			map(this->setPoint_y,0.0f,(float)this->height,(float)(this->image->height/2),(float)(this->image->height));
	;

	CvScalar whiteLine = cvScalar(255, 255, 255);
	CvScalar greenLine = cvScalar(0, 255, 0);
	CvScalar redLine = cvScalar(255, 0, 0);
	CvFont font;

	cvInitFont(&font, CV_FONT_HERSHEY_PLAIN, 1.0, 1.0, 0, 1, 8);

	if (sample == 0) {
		cvPutText(this->image, "Estimation", cvPoint(10, 12), &font, greenLine);
		cvPutText(this->image, "Set Point", cvPoint(120, 12), &font, redLine);
		cvPutText(this->image, "X", cvPoint(0, 12 + this->image->height / 4),
				&font, whiteLine);
		cvPutText(this->image, "Y",
				cvPoint(0, 12 + 3 * this->image->height / 4), &font, whiteLine);
		cvLine(this->image, cvPoint(0, this->image->height / 4),
				cvPoint(this->image->width, this->image->height / 4), whiteLine,
				0);
		cvLine(this->image, cvPoint(0, 0), cvPoint(0, this->image->height),
				whiteLine, 0);
	}

	cvCircle(this->image, cvPoint(this->sample, i_x_est), 0, greenLine, 0);
	cvCircle(this->image, cvPoint(this->sample, i_x_set), 0, redLine, 0);

	cvCircle(this->image, cvPoint(this->sample, i_y_est), 0, greenLine, 0);
	cvCircle(this->image, cvPoint(this->sample, i_y_set), 0, redLine, 0);

	this->sample++;

	if (sample >= this->image->width) {
		this->sample = 0;
		cvZero(this->image);
	}
	cvShowImage(this->window, this->image);
}

/** @} */
