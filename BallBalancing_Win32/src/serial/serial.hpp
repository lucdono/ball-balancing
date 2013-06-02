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
 * @file    Serial.hpp
 * @brief   Serial driver class.
 *
 * @note	Win32
 *
 * @addtogroup Serial
 * @{
 */

#ifndef SERIAL_HPP_
#define SERIAL_HPP_

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/

#include <windows.h>
#include <stdio.h>
#include <stdlib.h>

/*===========================================================================*/
/* Defines and Macros                                                        */
/*===========================================================================*/

/**
 * @brief		Serial timeout.
 */
#define WAIT_TIME 2000

/*===========================================================================*/
/* Types definitions.                                                        */
/*===========================================================================*/

/*
 * @brief    The serial driver class.
 */
class Serial {
private:
	HANDLE hSerial;			//!< Serial handler.
	bool connected;			//!< Connection status.
	COMSTAT status;			//!< Information about the connection
	DWORD errors;			//!< Last error

public:
	Serial(char *portName);
	~Serial();

	int ReadData(char *buffer, unsigned int nbChar);
	bool WriteData(char *buffer, unsigned int nbChar);
	bool IsConnected();
};

#endif /* SERIAL_HPP_ */

/** @} */
