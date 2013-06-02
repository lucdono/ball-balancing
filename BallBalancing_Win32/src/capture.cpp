/*
 * capture.cpp
 *
 *  Created on: 01/giu/2013
 *      Author: Luca
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#include <cv.h>
#include <highgui.h>

#include "compute.hpp"

#define WIN_BINARY_NAME		"Binary HSV Recognition"
#define WIN_FRAME_NAME		"Object Tracking"

static IplImage* imgTracking;
static int lastX = -1;
static int lastY = -1;
static bool doTrack = FALSE;

//This function threshold the HSV image and create a binary image
IplImage* GetThresholdedImage(IplImage* imgHSV) {
	IplImage* imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
	cvInRangeS(imgHSV, cvScalar(91, 43, 169), cvScalar(113, 256, 256),
			imgThresh);
	return imgThresh;
}

CvPoint trackObject(IplImage* imgThresh) {
	// Calculate the moments of 'imgThresh'
	CvMoments *moments = (CvMoments*) malloc(sizeof(CvMoments));
	cvMoments(imgThresh, moments, 1);
	double moment10 = cvGetSpatialMoment(moments, 1, 0);
	double moment01 = cvGetSpatialMoment(moments, 0, 1);
	double area = cvGetCentralMoment(moments, 0, 0);
	int posX = 0;
	int posY = 0;

	// if the area<1000, I consider that the there are no object in the image and it's because of the noise, the area is not zero
	if (area > 1000) {
		// calculate the position of the ball
		posX = moment10 / area;
		posY = moment01 / area;

		if (lastX >= 0 && lastY >= 0 && posX >= 0 && posY >= 0) {
			// Draw a yellow line from the previous point to the current point
			if (doTrack)
				cvLine(imgTracking, cvPoint(posX, posY), cvPoint(lastX, lastY),
						cvScalar(0, 0, 255), 4);
		}

		lastX = posX;
		lastY = posY;
	}

	free(moments);

	return cvPoint(posX, posY);
}

int main() {
	CvCapture* capture = 0;

	capture = cvCaptureFromCAM(1);
	if (!capture) {
		printf("Capture failure\n");
		return -1;
	}

	IplImage* frame = 0;

	cvNamedWindow(WIN_BINARY_NAME);
	cvNamedWindow(WIN_FRAME_NAME);

	frame = cvQueryFrame(capture);
	if (!frame){
		printf("Can not query frame\n");
		cvDestroyAllWindows();
		cvReleaseCapture(&capture);
		return -1;
	}else
		printf("Frame size %d x %d",frame->width,frame->height);

	Compute *compute = new Compute(frame->width,frame->height);
	if(!compute->IsConnected()){
		printf("Can not connect to serial device\n");
		cvDestroyAllWindows();
		cvReleaseCapture(&capture);
		return -1;
	}


	//create a blank image and assigned to 'imgTracking' which has the same size of original video
	imgTracking = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
	cvZero(imgTracking); //covert the image, 'imgTracking' to black

	//iterate through each frames of the video
	while (true) {

		frame = cvQueryFrame(capture);
		if (!frame)
			break;

		frame = cvCloneImage(frame);
		cvSmooth(frame, frame, CV_GAUSSIAN, 3, 3); //smooth the original image using Gaussian kernel

		IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3);
		cvCvtColor(frame, imgHSV, CV_BGR2HSV); //Change the color format from BGR to HSV
		IplImage* imgThresh = GetThresholdedImage(imgHSV);

		cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN, 3, 3); //smooth the binary image using Gaussian kernel

		cvShowImage(WIN_BINARY_NAME, imgThresh);

		CvPoint center = trackObject(imgThresh);

		// Add the tracking image and the frame
		if (doTrack)
			cvAdd(frame, imgTracking, frame);

		if (center.x != 0 && center.y != 0){
			cvCircle(frame, center, 50, CV_RGB(0,255,0), 4, 0);
			compute->Update(center.x, center.y);
		}

		cvShowImage(WIN_FRAME_NAME, frame);

		//Clean up used images
		cvReleaseImage(&imgHSV);
		cvReleaseImage(&imgThresh);
		cvReleaseImage(&frame);

		//Wait 50mS
		int c = cvWaitKey(10);
		//If 'ESC' is pressed, break the loop
		if ((char) c == 27)
			break;
		else if ((char) c == 't') {
			doTrack = !doTrack;
			cvZero(imgTracking);
		}
	}

	cvDestroyAllWindows();
	cvReleaseCapture(&capture);

	return 0;
}

