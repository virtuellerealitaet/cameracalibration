// *******************************************************************
// Stereo Calibration Tool for Stereo Sony Eye 3 Camera Setup
//
// code : Michael Stengel, 2017, 
// virtuellerealitaet@googlemail.com
// 
// 	implementation based on BOOK: Learning OpenCV: Computer Vision with the OpenCV Library
//  by Gary Bradski and Adrian Kaehler
//  Published by O'Reilly Media, October 3, 2008
//
// *******************************************************************

#include <stdafx.h>

#include <iostream>
#include <ctime>
#include <cstdlib>

#include "CameraPS3Eye.h"

using namespace cv;
using namespace std;

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
	);

// ***************************************************
// CAMERA DATA

static CameraPS3Eye *VidCapLeft;
static CameraPS3Eye *VidCapRight;

static bool newframe_left = false;
static bool newframe_right = false;
static cv::Mat current_frame_left;
static cv::Mat current_frame_right;

static void *userdata_left;
static void *userdata_right;


static cv::Mat left_roi;
static cv::Mat right_roi;
static Mat combined;


static bool switchcameras = false;

static int camera_width		= 320;
static int camera_height	= 240;
static float camera_fps		= 190.f;


static void cameracallback_left(cv::Mat frame, void *userdata)
{
	frame.copyTo(current_frame_left);
	newframe_left = true;
}

static void cameracallback_right(cv::Mat frame, void *userdata)
{
	frame.copyTo(current_frame_right);
	newframe_right = true;

}



bool startCameras()
{

	// list out the devices
	using namespace ps3eye;
	std::vector<PS3EYECam::PS3EYERef> devices(PS3EYECam::getDevices());
	LOGCON("Found %d cameras.\n", (int)devices.size());

	int numCams = (int)devices.size();

	if (numCams < 2) {
		cout << "No or not enough cameras for dual cam mode connected !" << endl;
		return false;
	}

	// create and initialize two sony ps3 eye cameras
	char windowName[64];



	VidCapLeft = new CameraPS3Eye("leftcam", 0);
	VidCapRight = new CameraPS3Eye("rightcam", 0);

	VidCapLeft->setCallback(cameracallback_left, userdata_left);
	VidCapRight->setCallback(cameracallback_right, userdata_right);


	//bool success = (VidCapLeft->initialize(640,480,3,60,0) && VidCapRight->initialize(640, 480, 3, 60, 1));
	bool success = (VidCapLeft->initialize(camera_width, camera_height, 3, camera_fps, 0) && VidCapRight->initialize(camera_width, camera_height, 3, camera_fps, 1));

	// adjust camera settings
	if (success)
	{
		VidCapLeft->startCapture();
		
		VidCapLeft->_autogain = true;
		VidCapLeft->_autowhitebalance = true;
		VidCapLeft->_flipVertically = true;
		VidCapLeft->_exposure = 20.f;
		VidCapLeft->updateCameraSettings();
		
		VidCapRight->startCapture();
		VidCapRight->_autogain = true;
		VidCapRight->_autowhitebalance = true;
		VidCapRight->_flipHorizontally = true;
		VidCapRight->_exposure = 200.f;
		VidCapRight->updateCameraSettings();
		
	}

	return success;
}

// update frames from both cameras
//void receiveCameraFrames()
//{
//
//	current_frame_left = VidCapLeft->receiveFrame();
//	current_frame_right = VidCapRight->receiveFrame();
//
//	transpose(current_frame_left, current_frame_left);
//	transpose(current_frame_right, current_frame_right);
//	
//	newframe_left = true;
//	newframe_right = true;
//
//}


int main(int argc, char** argv)
{


	combined = Mat(camera_height, 2 * camera_width, CV_8UC(3));
	left_roi = combined(Rect(0, 0, camera_width, camera_height));
	right_roi = combined(Rect(camera_width, 0, camera_width, camera_height));

	// start cameras
	if (!startCameras()) {
		cout << "Exiting.";
		return 0;
	}

	// register mouse event callback for new window
	//namedWindow("lefteye");
	//namedWindow("righteye");
	//namedWindow("combined");


	clock_t startTime = clock(); //Start timer
	double secondsPassed;
	double secondsToDelay = 1.0;

	long numFrames = 0;

	bool writeFrames = false;

	
	VideoWriter writer;
	
	if (writeFrames)
		writer.open("dualcam.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(2 * camera_width, camera_height), true);
	
	bool stop = false;

	while (!stop)
	{

		//receiveCameraFrames();

		if (newframe_left && newframe_right)
		{

			if (writeFrames)
			{
				current_frame_left.copyTo(left_roi);
				current_frame_right.copyTo(right_roi);
				writer << combined;
			}

			newframe_left = false;
			newframe_right = false;
			

			//numFrames++;

			//imshow("combined", combined);
			//imshow("lefteye", left_roi);
			//imshow("righteye", right_roi);
			//waitKey(1);

			//secondsPassed = (clock() - startTime) / CLOCKS_PER_SEC;
			//if (secondsPassed >= 1.0)
			//{
			//	std::cout << numFrames << " hz" << std::endl;
			//	numFrames = 0;
			//	startTime = clock();
			//}

			if (GetAsyncKeyState(VK_SPACE))
				stop = true;

		}
		

	}

	if (writeFrames)
		writer.release();

	destroyAllWindows();

	// deinitialize both cameras	
	VidCapLeft->stopCapture();
	VidCapRight->stopCapture();

	// run validation by image rectification
	return 0;

}
