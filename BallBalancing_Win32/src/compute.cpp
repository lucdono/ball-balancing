/*
 * compute.cpp
 *
 *  Created on: 02/giu/2013
 *      Author: Luca
 */

#include "serial.hpp"
#include "compute.hpp"
#include "stdint.h"

Compute::Compute(int width, int height) {
	this->serial = new Serial("\\\\.\\COM19");
	this->pidX = new PID(0.1f, 0.1f, -0.05f);
	this->pidY = new PID(0.05f, 0.1f, -0.02f);
	this->kalmanX = new Kalman(0.02f, 0.015f);
	this->kalmanY = new Kalman(0.02f, 0.015f);
	this->width = width;
	this->height = height;
}

Compute::~Compute() {

}

bool Compute::IsConnected() {
	return this->serial->IsConnected();
}
void Compute::Update(int x, int y) {
	char data[2];
	int setpoint_x = this->width / 2;
	int setpoint_y = this->height / 2;

	if (serial->IsConnected()) {
		float x_est = this->kalmanX->update((float) x);
		float y_est = this->kalmanY->update((float) y);

		float ctrl_x = this->pidX->updatePID(setpoint_x, x_est);
		float ctrl_y = this->pidY->updatePID(setpoint_y, y_est);

		data[0] =
				map(ctrl_x,90.0f,-90.0f,-(float)(this->width / 2),(float)(this->width / 2));
		data[1] =
				map(ctrl_y,90.0f,-90.0f,-(float)(this->width / 2),(float)(this->width / 2));

		serial->WriteData(data, 2);
	} else
		printf("Serial port unavailable!\n");
}
