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
 * @file    Capture.cpp
 * @brief   Main program.
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

#include "capture.hpp"
#include "compute.hpp"
#include "math.h"

/*===========================================================================*/
/* Variables.                                                                */
/*===========================================================================*/

/**
 * @brief		The main computing class instanace.
 */
static Compute *compute;

/**
 * @brief		Image tracking frame.
 */
static IplImage* imgTracking;

/**
 * @brief		Plot frame.
 */
static IplImage* imgPlot;

/**
 * @brief		Tracking last X position.
 */
static int lastX = -1;

/**
 * @brief		Tracking last Y position.
 */
static int lastY = -1;

/**
 * @brief		Tracking enablement.
 */
static bool doTrack = FALSE;

/**
 * @brief		Set the setpoint along a cirlce.
 */
static bool doRotate = FALSE;

/*===========================================================================*/
/* Internal functions.                                                       */
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
	cvInRangeS(imgHSV, HSV_MIN, HSV_MAX, imgThresh);
	return imgThresh;
}

/**
 * @brief		Object tracking functions from a binary image.
 *
 * @param[in]	imgThresh	the binary image.
 *
 * @return		the center of gravity of the tracked object.
 */
CvPoint trackObject(IplImage* imgThresh) {
	/*
	 * Calculate the moments of 'imgThresh'
	 */
	CvMoments *moments = (CvMoments*) malloc(sizeof(CvMoments));
	cvMoments(imgThresh, moments, 1);
	double moment10 = cvGetSpatialMoment(moments, 1, 0);
	double moment01 = cvGetSpatialMoment(moments, 0, 1);
	double area = cvGetCentralMoment(moments, 0, 0);
	int posX = 0;
	int posY = 0;

	/*
	 * If the area < 1000, there are no object in the image
	 * and it's because of the noise, the area is not zero
	 */
	if (area > 1000) {
		/*
		 * calculate the position of the ball
		 */
		posX = moment10 / area;
		posY = moment01 / area;

		if (lastX >= 0 && lastY >= 0 && posX >= 0 && posY >= 0) {
			if (doTrack)
				cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY),
						TRACKING_COLOR, 4);
		}

		lastX = posX;
		lastY = posY;
	}

	free(moments);

	return cvPoint(posX, posY);
}

/*===========================================================================*/
/* External functions.                                                       */
/*===========================================================================*/

/**
 * @brief		The program entry point.
 */
int main() {
	/*
	 * The rotation angle around the center
	 */
	float angle = 0.0f;

	CvCapture* capture = 0;

	/*
	 * Start camera
	 */
	capture = cvCaptureFromCAM(CAMERA_ID);
	if (!capture) {
		printf("Capture failure\n");
		return -1;
	}

	IplImage* frame = 0;

	/*
	 * Initialize windows
	 */
	cvNamedWindow(WIN_BINARY_NAME);
	cvNamedWindow(WIN_FRAME_NAME);
	cvNamedWindow(WIN_GRAPH_NAME);

	frame = cvQueryFrame(capture);
	if (!frame) {
		printf("Can not query frame\n");
		cvDestroyAllWindows();
		cvReleaseCapture(&capture);
		return -1;
	}


	/*
	 * Initialize computing algorithm class
	 */
	compute = new Compute(frame->width, frame->height);
	if (!compute->IsConnected()) {
		printf("Can not connect to serial device\n");
		cvDestroyAllWindows();
		cvReleaseCapture(&capture);
		return -1;
	}

	/*
	 * Create a blank image and assigned to 'imgTracking' which has
	 * the same size of original video
	 */
	imgTracking = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvZero(imgTracking); //covert the image, 'imgTracking' to black

	/*
	 * Create a blank image for signal plot and inizialize Compute class
	 */
	imgPlot = cvCreateImage(cvSize(800,200), IPL_DEPTH_8U, 3);
	cvZero(imgPlot); //covert the image, 'imgTracking' to black
	compute->SetPlotter(WIN_GRAPH_NAME,imgPlot);
	cvShowImage(WIN_GRAPH_NAME, imgPlot);

	/*
	 * Iterate through each frames of the video
	 */
	while (true) {

		frame = cvQueryFrame(capture);
		if (!frame)
			break;

		frame = cvCloneImage(frame);
		/*
		 * Smooth the original image and binary image using Gaussian kernel,
		 * then change the color format from BGR to HSV
		 */
		cvSmooth(frame, frame, CV_GAUSSIAN, 3, 3);

		IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
		cvCvtColor(frame, imgHSV, CV_BGR2HSV);
		IplImage* imgThresh = GetThresholdedImage(imgHSV);

		cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN, 3, 3);

		/*
		 * use cvResize to resize source to a destination image
		 */
		int percent = BINARY_RESIZE;
		IplImage *destination = cvCreateImage(
				cvSize((int) ((imgThresh->width * percent) / 100),
						(int) ((imgThresh->height * percent) / 100)),
				imgThresh->depth, imgThresh->nChannels);
		cvResize(imgThresh, destination);
		cvShowImage(WIN_BINARY_NAME, destination);

		CvPoint center = trackObject(imgThresh);

		/*
		 * Add the tracking image and the frame
		 */
		if (doTrack)
			cvAdd(frame, imgTracking, frame);

		if (doRotate) {
			/*
			 * Describe a circle around the center.
			 */
			angle += ROTATION_ANGLE_STEP;
			if (angle > 360.0f)
				angle = 0.0f;

			int x = ROTATION_RADIUS * cos(angle * M_PI / 180.0)
					+ frame->width / 2;
			int y = ROTATION_RADIUS * sin(angle * M_PI / 180.0)
					+ frame->height / 2;

			compute->setPoint(x, y);
		} else
			/*
			 * Reset the set point to the center.
			 */
			compute->center();

		if (center.x != 0 && center.y != 0) {
			cvCircle(frame, center, 50, CV_RGB(0,255,0), 4, 0);
			/*
			 * Control algorithm update
			 */
			compute->Update(center.x, center.y);
		}

		cvShowImage(WIN_FRAME_NAME, frame);

		/*
		 * Clean up used images
		 */
		cvReleaseImage(&imgHSV);
		cvReleaseImage(&imgThresh);
		cvReleaseImage(&frame);

		/*
		 * Wait 10mS
		 */
		int c = cvWaitKey(10);

		/*
		 * If 'ESC' is pressed, break the loop
		 */
		if ((char) c == 27)
			break;
		/*
		 * If 't' pressed draw tracking position
		 */
		else if ((char) c == 't') {
			doTrack = !doTrack;
			cvZero(imgTracking);
		}
		/*
		 * If 'r' pressed move object around the center
		 */
		else if ((char) c == 'r') {
			doRotate = !doRotate;
		}
	}

	/*
	 * Release resources
	 */
	cvDestroyAllWindows();
	cvReleaseCapture(&capture);

	return 0;
}

/** @} */
