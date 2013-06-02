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
 * @file    Serial.cpp
 * @brief   Serial driver class implementation.
 *
 * @note	Win32
 *
 * @addtogroup Serial
 * @{
 */

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/

#include "serial.hpp"

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

/**
 * @brief		The Constructor
 * @details		Initialize Serial communication with the given COM port.
 *
 * @param[in]	portName	The serial port to initializa.
 */
Serial::Serial(char *portName) {
	this->connected = false;
	this->errors = 0;

	/*
	 * Connect to the given port through CreateFile
	 */
	this->hSerial = CreateFile(portName, GENERIC_READ | GENERIC_WRITE, 0, NULL,
			OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

	/*
	 * Check if the connection was succeeded
	 */
	if (this->hSerial == INVALID_HANDLE_VALUE) {
		/*
		 * If not success full display an Error
		 */
		if (GetLastError() == ERROR_FILE_NOT_FOUND) {
			printf(
					"ERROR: Handle was not attached. Reason: %s not available.\n",
					portName);
		} else {
			printf("ERROR!!!");
		}
	} else {
		/*
		 * Set the comm parameters
		 */
		DCB dcbSerialParams = { 0 };

		/*
		 * Try to get the current state
		 */
		if (!GetCommState(this->hSerial, &dcbSerialParams)) {
			printf("failed to get current serial parameters!");
		} else {
			/*
			 * Define serial connection parameters for STM32F0 board
			 */
			dcbSerialParams.BaudRate = CBR_115200;
			dcbSerialParams.ByteSize = 8;
			dcbSerialParams.StopBits = ONESTOPBIT;
			dcbSerialParams.Parity = NOPARITY;

			/*
			 * Set the parameters and check for their proper application
			 */
			if (!SetCommState(hSerial, &dcbSerialParams)) {
				printf("ALERT: Could not set Serial Port parameters");
			} else {
				/*
				 * If everything went fine we're connected
				 */
				this->connected = true;

				Sleep (WAIT_TIME);
			}
		}
	}
}

/**
 * @brief		The destructor.
 * @details		Close the serial connection.
 */
Serial::~Serial() {
	if (this->connected) {
		this->connected = false;
		CloseHandle(this->hSerial);
	}
}

/**
 * @brief		Read data from serial port.
 *
 * @param[in]	buffer	the destination buffer.
 * @param[in]	nbChar	the number of bytes to read.
 *
 * @return		the number of bytes read.
 * @retval		-1	if nothing to read.
 */
int Serial::ReadData(char *buffer, unsigned int nbChar) {
	DWORD bytesRead;
	unsigned int toRead;

	ClearCommError(this->hSerial, &this->errors, &this->status);

	/*
	 * Check if there is something to read
	 */
	if (this->status.cbInQue > 0) {
		/*
		 * If there is we check if there is enough data to read the required
		 * number of characters, if not we'll read only the available
		 * characters.
		 */
		if (this->status.cbInQue > nbChar) {
			toRead = nbChar;
		} else {
			toRead = this->status.cbInQue;
		}

		if (ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL)
				&& bytesRead != 0) {
			return bytesRead;
		}

	}

	return -1;
}

/**
 * @brief		Write data to serial port.
 *
 * @param[in]	buffer	the source buffer.
 * @param[in]	nbChar	the number of bytes to write.
 *
 * @return		TRUE if write operation succeeded, FALSE otherwise.
 */
bool Serial::WriteData(char *buffer, unsigned int nbChar) {
	DWORD bytesSend;

	/*
	 * Write the buffer on the Serial port
	 */
	if (!WriteFile(this->hSerial, (void *) buffer, nbChar, &bytesSend, 0)) {
		ClearCommError(this->hSerial, &this->errors, &this->status);
		return false;
	} else
		return true;
}

/**
 * @brief		Get the connection status.
 *
 * @return		TRUE if serial port has been connected, FALSE otherwise.
 */
bool Serial::IsConnected() {
	return this->connected;
}

/** @} */
