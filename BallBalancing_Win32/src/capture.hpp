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
 * @file    Capture.hpp
 * @brief   Main program header.
 *
 * @note	Win32
 *
 * @addtogroup Main
 * @{
 */

#ifndef CAPTURE_HPP_
#define CAPTURE_HPP_

/*===========================================================================*/
/* Defines and Macros                                                        */
/*===========================================================================*/

/**
 * @brief		The camera id used (0 = primary, 1 = secondary)
 */
#define CAMERA_ID			1

/**
 * @brief		Binary frame window name.
 */
#define WIN_BINARY_NAME		"Binary HSV Recognition"

/**
 * @brief		Object tracking frame window name.
 */
#define WIN_FRAME_NAME		"Object Tracking"

/**
 * @brief		Graph window name.
 */
#define WIN_GRAPH_NAME		"Signal plot"

/**
 * @brief		Scaling factor (%) of the binary frame window
 */
#define	BINARY_RESIZE		50
/**
 * @brief		Lower HSV values for creating binary image.
 */
#define HSV_MIN				cvScalar(42, 140, 43)

/**
 * @brief		Upper HSV values for creating binary image.
 */
#define HSV_MAX				cvScalar(1118, 256, 256)

/**
 * @brief		Color for the tracking path.
 */
#define TRACKING_COLOR		cvScalar(0, 0, 255)

/**
 * @brief		The radius of the circle path around the center.
 */
#define ROTATION_RADIUS		150.0f

/**
 * @brief		The rotation angle step around the center.
 */
#define ROTATION_ANGLE_STEP	2.0f

#endif /* CAPTURE_HPP_ */

/** @} */
