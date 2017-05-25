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

//#include "CameraPS3Eye.h"
#include "ThreadClass.h"

using namespace cv;
using namespace std;

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
	);

// ***************************************************
// CAMERA DATA

const int NUM_CAMERAS = 3;

static ThreadCamera *VidCapCam[NUM_CAMERAS];

static bool newframe[NUM_CAMERAS];
static cv::Mat current_frame[NUM_CAMERAS];

static cv::Mat roi[NUM_CAMERAS];
static Mat combined;

bool writeFrames = false;
float writeFPS = 30.f;

static void cameracallback(cv::Mat frame, void *userdata)
{
	int camIndex = (int)userdata;
	//std::cout << camIndex << std::endl;

	frame.copyTo(current_frame[camIndex]);
	newframe[camIndex] = true;

	//imshow(std::to_string(camIndex), current_frame[camIndex]);
	//waitKey(1);

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
		VidCapCam[camIdx] = new ThreadCamera();

		VidCapCam[camIdx]->setCallback(cameracallback, (void*)camIdx);
		
		// init cam 0 and cam 1 using VGA@30Hz
		float camera_fps = 125;
		int camera_width = 320;
		int camera_height = 240;

		//float camera_fps = 195;
		//int camera_width = 320;
		//int camera_height = 240;
				
		//if (camIdx > 1) // init cam 2 and cam 3 using QVGA@100Hz
		//{
		//	camera_fps = 100.0;
		//	camera_width = 320;
		//	camera_height = 240;
		//}
		
		// init cameras and adjust camera settings
		bool success = VidCapCam[camIdx]->initialize(camIdx, camera_width, camera_height, 3, camera_fps);
		
		if (!success) // break if initialization fails
			return false;
	}
	for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
	{
		VidCapCam[camIdx]->startCapture();
		
		Sleep(500);
		
		VidCapCam[camIdx]->_autogain = true;
		VidCapCam[camIdx]->_autowhitebalance = true;
		VidCapCam[camIdx]->_flipVertically = true;
		VidCapCam[camIdx]->_exposure = 50.f;
		VidCapCam[camIdx]->updateCameraSettings();

		Sleep(500);
		
	}
	

	return true;
}

int main(int argc, char** argv)
{

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
	
	std::cout << "combined size " << maxHeight << " x " << totalWidth << endl;

	// open camera writer if required
	VideoWriter writer;
	if (writeFrames)
		writer.open("multicam.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), writeFPS, cv::Size(totalWidth, maxHeight), true);
	
	// region of interest per camera
	int currentX = 0;
	for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
	{

		std::cout << "roi" << camIdx << " : " << currentX << endl;

		roi[camIdx] = combined(Rect(currentX, 0, VidCapCam[camIdx]->getCameraWidth(), VidCapCam[camIdx]->getCameraHeight()));
		currentX += VidCapCam[camIdx]->getCameraWidth();
	}
	
	// register mouse event callback for new window
	//namedWindow("combined");
	
	long long startTime = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
	int writeInterval = 1000.0 / writeFPS;
	std::chrono::milliseconds msPassed;

	
	// main loop
	
	bool stop = false;
	while (!stop)
	{
		{
			
			if (writeFrames)
			{
				long long now = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
				long long milliSecondsPassed = now - startTime;

				if (milliSecondsPassed >= writeInterval)
				{
					startTime = now;


					// fill region of interests in combined frame
					for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
						current_frame[camIdx].copyTo(roi[camIdx]);

					// reset camera frame status
					for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
						newframe[camIdx] = false;

					// write frame using video writer
					writer << combined;
					//std::cout << "writing frame" << std::endl;

					std::this_thread::sleep_for(std::chrono::milliseconds(max(0, writeInterval - 1)));

				}
			}
			else
			{

				// fill region of interests in combined frame
				for (int camIdx = 0; camIdx < NUM_CAMERAS; camIdx++)
					current_frame[camIdx].copyTo(roi[camIdx]);

				imshow("combined", combined);
				waitKey(1);

				//std::this_thread::sleep_for(std::chrono::seconds(1));

			}
		}

		//std::this_thread::sleep_for(std::chrono::seconds(1));
		//std::this_thread::sleep_for(std::chrono::milliseconds(10));

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

	return 0;

}
