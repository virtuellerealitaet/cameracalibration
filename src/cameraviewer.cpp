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

const int NUM_CAMERAS = 4;

static CameraPS3Eye *VidCapCam[NUM_CAMERAS];
static bool newframe[NUM_CAMERAS];
static cv::Mat current_frame[NUM_CAMERAS];

static cv::Mat roi[NUM_CAMERAS];
static Mat combined;


//static int camera_width		= 320;
//static int camera_height	= 240;
//static float camera_fps		= 190.f;

static int camera_width		= 640;
static int camera_height	= 480;
static float camera_fps		= 60.f;

static void cameracallback(cv::Mat frame, void *userdata)
{
	int camIndex = (int)userdata;
	//std::cout << camIndex << std::endl;

	frame.copyTo(current_frame[camIndex]);
	newframe[camIndex] = true;
}


bool startCameras()
{

	// list out the devices
	using namespace ps3eye;
	std::vector<PS3EYECam::PS3EYERef> devices(PS3EYECam::getDevices());
	LOGCON("Found %d cameras.\n", (int)devices.size());

	int numCams = (int)devices.size();

	if (numCams < NUM_CAMERAS) {
		cout << "No or not enough cameras for cam mode connected ! Required : " << NUM_CAMERAS << endl;
		return false;
	}

	// create and initialize two sony ps3 eye cameras
	for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
	{
		VidCapCam[camIdx] = new CameraPS3Eye(camIdx);
		VidCapCam[camIdx]->setCallback(cameracallback, (void*)camIdx);

		
		//if (camIdx == 0 || camIdx == 2)
		if (camIdx < 2)
		{
			camera_fps = 30.0;
			camera_width = 640;
			camera_height = 480;
		} 
		else
		{
			camera_fps = 100.0;
			camera_width = 320;
			camera_height = 240;
		}
		


		// init cameras and adjust camera settings
		bool success = VidCapCam[camIdx]->initialize(camera_width, camera_height, 3, camera_fps, camIdx);

		if (!success)
			return false;
	}
	for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
	{
		

		
		VidCapCam[camIdx]->startCapture();
		
		Sleep(100);
		
		VidCapCam[camIdx]->_autogain = true;
		VidCapCam[camIdx]->_autowhitebalance = true;
		VidCapCam[camIdx]->_flipVertically = true;
		VidCapCam[camIdx]->_exposure = 50.f;
		VidCapCam[camIdx]->updateCameraSettings();

		Sleep(100);
		
	}
	

	return true;
}

int main(int argc, char** argv)
{
	
	camera_width = 640;
	camera_height = 480;


	VideoWriter writer;
	bool writeFrames = true;

	if (writeFrames)
		//writer.open("multicam.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(NUM_CAMERAS * camera_width / 2, camera_height / 2), true);
		writer.open("multicam.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(3 * camera_width, camera_height), true);


	//combined = Mat(camera_height, NUM_CAMERAS * camera_width, CV_8UC(3));
	combined = Mat(camera_height, 3 * camera_width, CV_8UC(3));

	//cv::Mat combined2 = Mat(camera_height / 2, NUM_CAMERAS * camera_width / 2, CV_8UC(3));

	//for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
	//	roi[camIdx] = combined(Rect(camIdx * camera_width, 0, camera_width, camera_height));

	roi[0] = combined(Rect(0, 0, 640, 480));
	roi[1] = combined(Rect(640, 0, 640, 480));
	roi[2] = combined(Rect(2*640, 0, 320, 240));
	roi[3] = combined(Rect(2*640+320, 0, 320, 240));

	// start cameras
	if (!startCameras()) {
		cout << "Exiting.";
		return 0;
	}

	// register mouse event callback for new window
	//namedWindow("combined");


	clock_t startTime = clock(); //Start timer
	double secondsPassed;
	double secondsToDelay = 1.0;
	long numFrames = 0;


	
	bool stop = false;
	while (!stop)
	{

		//receiveCameraFrames();

		bool allFramesNew = true;

		for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
			if (!newframe[camIdx])
				allFramesNew = false;

		if (allFramesNew)
		{

			if (writeFrames)
			{
				for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
					current_frame[camIdx].copyTo(roi[camIdx]);
				
				
				//cv::resize(combined, combined2, combined2.size());
				writer << combined;
			}

			// reset camera frame status
			for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
				newframe[camIdx] = false;

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

			// stop capture after pressing space bar
			if (GetAsyncKeyState(VK_SPACE))
				stop = true;

		}
		

	}

	// release frame writer
	if (writeFrames)
		writer.release();

	// release opencv windows
	destroyAllWindows();

	// deinitialize cameras	
	for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
		VidCapCam[camIdx]->stopCapture();


	return 0;

}
