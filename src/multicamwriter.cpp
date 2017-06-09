// *******************************************************************
// Camera Viewer for multiple Sony Eye 3 Cameras
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

#include "ThreadCamera.h"

using namespace cv;
using namespace std;



// ***************************************************
// CAMERA DATA

const int MAX_NUM_CAMERAS	= 4;
int numDetectedCameras		= 0;

static ThreadCamera			*VidCapCam[MAX_NUM_CAMERAS];
static bool					newframe[MAX_NUM_CAMERAS];
static cv::Mat				current_frame[MAX_NUM_CAMERAS];

static cv::Mat				roi[MAX_NUM_CAMERAS];
static Mat					combined;

bool writeFrames			= true;
float writeFPS				= 30.f;

static void cameracallback(cv::Mat frame, void *userdata)
{
	int camIndex = *((int*)(&userdata)); // precision loss possible !

	frame.copyTo(current_frame[camIndex]);
	newframe[camIndex] = true;
}

bool startCameras()
{

	// list out the devices
	using namespace ps3eye;
	std::vector<PS3EYECam::PS3EYERef> devices(PS3EYECam::getDevices());
	LOGCON("Found %d cameras.\n", (int)devices.size());

	numDetectedCameras = devices.size();

	if (numDetectedCameras < 1) {
		cout << "No sony eye camera detected !" << endl;
		return false;
	}

	// create and initialize two sony ps3 eye cameras
	for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
	{
		//VidCapCam[camIdx] = new CameraPS3Eye(camIdx);
		VidCapCam[camIdx] = new ThreadCamera();

		VidCapCam[camIdx]->setCallback(cameracallback, (void*)camIdx);
				
		int camera_width = 640;
		int camera_height = 480;
		float camera_fps = 30;

		// init cam 0 and cam 1 using VGA@30Hz
		//float camera_fps = 125;
		//int camera_width = 320;
		//int camera_height = 240;

		//float camera_fps = 195;
		//int camera_width = 320;
		//int camera_height = 240;
				
		//if (camIdx >= 1) // init cam 2 and cam 3 using QVGA@100Hz
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
	for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
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
		cout << "Could not start camera(s). Exiting.";
		return 0;
	}
	
	// estimate size of combined frame for visualization and video writer
	int maxHeight = 0;
	int totalWidth = 0;
	for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
	{
		if (VidCapCam[camIdx]->getCameraHeight() > maxHeight)
			maxHeight = VidCapCam[camIdx]->getCameraHeight();
		totalWidth += VidCapCam[camIdx]->getCameraWidth();
	}
	combined = Mat(maxHeight, totalWidth, CV_8UC(3));
	
	std::cout << "Combined sized for output video : " << totalWidth<< " x " << maxHeight << endl;

	// open camera writer if required
	std::cout << "Creating video writer (writing into multicam.avi)" << endl;
		 
	VideoWriter writer;
	if (writeFrames)
		writer.open("multicam.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), writeFPS, cv::Size(totalWidth, maxHeight), true);
	
	// region of interest per camera
	int currentX = 0;
	for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
	{

		std::cout << "roi" << camIdx << " : " << currentX << endl;

		roi[camIdx] = combined(Rect(currentX, 0, VidCapCam[camIdx]->getCameraWidth(), VidCapCam[camIdx]->getCameraHeight()));
		currentX += VidCapCam[camIdx]->getCameraWidth();
	}
	
	long long startTime = chrono::duration_cast<chrono::milliseconds>(chrono::steady_clock::now().time_since_epoch()).count();
	int writeInterval = 1000.0 / writeFPS;
	std::chrono::milliseconds msPassed;

	
	// main loop
	
	std::cout << "Begin camera capturing & video writing (press key to stop process)" << endl;

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
					for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
						current_frame[camIdx].copyTo(roi[camIdx]);

					// reset camera frame status
					for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
						newframe[camIdx] = false;

					// write frame using video writer
					writer << combined;
	
					std::this_thread::sleep_for(std::chrono::milliseconds(max(0, writeInterval - 1)));

				}
			}
			//else
			{

				// fill region of interests in combined frame
				for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
					current_frame[camIdx].copyTo(roi[camIdx]);

				imshow("combined", combined);
				waitKey(1);

			}
		}

		// quite program on keyboard input
		char c = cvWaitKey(1);
		if (c != -1)
			stop = true;
		

	}

	cout << "\nCamera loop stopped.\n\nClosing video....";

	// release frame writer
	if (writeFrames)
		writer.release();

	cout << "done.\nDisconnecting cameras ...";

	// release opencv windows
	destroyAllWindows();

	// deinitialize cameras	
	for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
		VidCapCam[camIdx]->stopCapture();

	cout << "done.\n\nRegular program exit.\n\n";

	return 0;

}
