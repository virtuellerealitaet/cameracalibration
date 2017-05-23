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
#include "ThreadClass.h"

using namespace cv;
using namespace std;

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
	);

// ***************************************************
// CAMERA DATA

const int NUM_CAMERAS = 3;

//static CameraPS3Eye *VidCapCam[NUM_CAMERAS];
static ThreadCamera *VidCapCam[NUM_CAMERAS];

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

bool writeFrames = true;

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
		//VidCapCam[camIdx] = new CameraPS3Eye(camIdx);
		//VidCapCam[camIdx]->setCallback(cameracallback, (void*)camIdx);

		VidCapCam[camIdx] = new ThreadCamera(camIdx);
		
		// init cam 0 and cam 1 using VGA@30Hz
		if (camIdx < 2)
		{
			camera_fps = 30.0;
			camera_width = 640;
			camera_height = 480;
		} 
		else // init cam 2 and cam 3 using QVGA@100Hz
		{
			camera_fps = 100.0;
			camera_width = 320;
			camera_height = 240;
		}
		


		// init cameras and adjust camera settings
		//bool success = VidCapCam[camIdx]->initialize(camera_width, camera_height, 3, camera_fps, camIdx);
		bool success = VidCapCam[camIdx]->initialize();

		if (!success) // break if initialization fails
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

	//MyClass instance;
	//instance.Start();
	

	// start cameras
	if (!startCameras())
	{
		cout << "Exiting.";
		return 0;
	}
	
	// estimate size of combined frame for visualization and video writer
	int maxHeight = 0;
	int totalWidth = 0;
	for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
	{
		if (VidCapCam[camIdx]->getCameraHeight() > maxHeight)
			maxHeight = VidCapCam[camIdx]->getCameraHeight();
		totalWidth += VidCapCam[camIdx]->getCameraWidth();
	}
	combined = Mat(maxHeight, totalWidth, CV_8UC(3));
	
	// open camera writer if required
	VideoWriter writer;
	if (writeFrames)
		writer.open("multicam.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 30.0, cv::Size(totalWidth, maxHeight), true);
	
	// region of interest per camera
	int currentX = 0;
	for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
	{
		roi[camIdx] = combined(Rect(currentX, 0, VidCapCam[camIdx]->getCameraWidth(), VidCapCam[camIdx]->getCameraHeight()));
		currentX += VidCapCam[camIdx]->getCameraWidth();
	}



	// register mouse event callback for new window
	//namedWindow("combined");
	
	clock_t startTime = clock(); //Start timer
	double secondsPassed;
	double secondsToDelay = 1.0;
	long numFrames = 0;
	
	// main loop
	
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
				// fill region of interests in combined frame
				for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
					current_frame[camIdx].copyTo(roi[camIdx]);

				// write frame using video writer
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

		}

		std::this_thread::sleep_for(std::chrono::seconds(1));

		// stop capture after pressing space bar
		if (GetAsyncKeyState(VK_SPACE))
			stop = true;
		

	}

	// release frame writer
	if (writeFrames)
		writer.release();

	// release opencv windows
	destroyAllWindows();

	// deinitialize cameras	
	for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
		VidCapCam[camIdx]->stopCapture();

	//instance.Stop();


	return 0;

}
