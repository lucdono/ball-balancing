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
 * @file    findHSVValues.cpp
 * @brief   Main program for finding correct values of HSV.
 * @details More details at http://opencv-srf.blogspot.it/2010/09/object-detection-using-color-seperation.html
 *
 * @note	Win32
 *
 * @addtogroup Main
 * @{
 */

/*===========================================================================*/
/* Include files.                                                            */
/*===========================================================================*/

#include <cv.h>
#include <highgui.h>

/*===========================================================================*/
/* Internal Variables.                                                       */
/*===========================================================================*/

static int lowerH = 0;		//!< Lower value of Hue
static int lowerS = 0;		//!< Lower value of Saturation
static int lowerV = 0;		//!< Lower value of Brightness

static int upperH = 180;	//!< Upper value of Hue
static int upperS = 256;	//!< Upper value of Saturation
static int upperV = 256;	//!< Upper value of Brightness

/*===========================================================================*/
/* Internal Functions.                                                       */
/*===========================================================================*/

/**
 * @brief		Threshold the HSV image and create a binary image
 *
 * @param[in]	imgHSV	the HSV frame.
 *
 * @return		the binary image.
 */
IplImage* GetThresholdedImage(IplImage* imgHSV) {

	IplImage* imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
	cvInRangeS(imgHSV, cvScalar(lowerH, lowerS, lowerV),
			cvScalar(upperH, upperS, upperV), imgThresh);

	return imgThresh;

}

/**
 * @brief		This function create two windows and 6 trackbars.
 */
void setwindowSettings(void) {
	cvNamedWindow("Video");
	cvNamedWindow("Ball");

	cvCreateTrackbar("LowerH", "Ball", &lowerH, 180, NULL);
	cvCreateTrackbar("UpperH", "Ball", &upperH, 180, NULL);

	cvCreateTrackbar("LowerS", "Ball", &lowerS, 256, NULL);
	cvCreateTrackbar("UpperS", "Ball", &upperS, 256, NULL);

	cvCreateTrackbar("LowerV", "Ball", &lowerV, 256, NULL);
	cvCreateTrackbar("UpperV", "Ball", &upperV, 256, NULL);
}

/*===========================================================================*/
/* External functions.                                                       */
/*===========================================================================*/

/**
 * @brief		The program entry point.
 */
int main() {
	CvCapture* capture = 0;

	capture = cvCaptureFromCAM(1);
	if (!capture) {
		printf("Capture failure\n");
		return -1;
	}

	IplImage* frame = 0;

	setwindowSettings();

	//iterate through each frames of the video
	while (true) {

		frame = cvQueryFrame(capture);
		if (!frame)
			break;
		frame = cvCloneImage(frame);

		IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
		cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV

		IplImage* imgThresh = GetThresholdedImage(imgHSV);

		cvShowImage("Ball", imgThresh);
		cvShowImage("Video", frame);

		//Clean up used images
		cvReleaseImage(&imgHSV);
		cvReleaseImage(&imgThresh);
		cvReleaseImage(&frame);

		//Wait 80mS
		int c = cvWaitKey(80);
		//If 'ESC' is pressed, break the loop
		if ((char) c == 27)
			break;

	}

	cvDestroyAllWindows();
	cvReleaseCapture(&capture);

	return 0;
}

/** @} */
