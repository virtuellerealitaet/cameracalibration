
#include "stdafx.h"
#include "ThreadCamera.h"

#include "argtable3.h"


const int MAX_NUM_CAMERAS = 2;

int numDetectedCameras = 0;

static ThreadCamera *VideoCapCam[MAX_NUM_CAMERAS];
static bool newframe[MAX_NUM_CAMERAS];
static cv::Mat current_frame[MAX_NUM_CAMERAS];
static cv::Mat roi[MAX_NUM_CAMERAS];

static void cameracallback(cv::Mat frame, void *userdata)
{
	int camIndex = *((int*)(&userdata)); // precision loss possible !

    frame.copyTo(current_frame[camIndex]);
    newframe[camIndex] = true;

}

bool startCameras()
{
	
#ifdef UNIX
    if (!PS3EYECam::setupDevices())
    {
        printf("Initialization of Sony Eye Cam(s) failed. Exiting..\n");
        return 0;
    }
#endif

#ifdef WIN32
	using namespace ps3eye;
#endif

    std::vector<PS3EYECam::PS3EYERef> devices = PS3EYECam::getDevices();

    if (devices.size() < 1)
    {
        printf("No sony eye cam connected ! Exiting..\n");
        return false;
    }


    // load an image

    int framecounter = 0;

	numDetectedCameras = devices.size();
    
    for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
    {
        VideoCapCam[camIdx] = new ThreadCamera();

        VideoCapCam[camIdx]->setCallback(cameracallback, (void*)camIdx);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        bool success = VideoCapCam[camIdx]->initialize(camIdx, 320, 240, 3, 187);
        //bool success = VideoCapCam[camIdx]->initialize(camIdx, 640, 480, 3, 30);
        if (!success)
            return false;

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
	
    for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
    {
        printf("starting camera %d\n", camIdx);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        VideoCapCam[camIdx]->startCapture();

    }


    return true;

}

bool stopCameras()
{
    for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
    {
        VideoCapCam[camIdx]->stopCapture();

        printf("stopping camera %d\n", camIdx);
    }

    for (int camIdx = 0; camIdx < numDetectedCameras; camIdx++)
    {
        delete VideoCapCam[camIdx];

        printf("deleting camera %d\n", camIdx);
    }

    return true;
}

int main(int argc, char *argv[])
{

    if (!startCameras())
        return 0;

    std::vector<std::string> windowNames;

    for (int camIndex = 0; camIndex < numDetectedCameras; camIndex++)
    {
        std::string windowName = "camera" + std::to_string(camIndex);
        windowNames.push_back(windowName);
        cv::namedWindow(windowName); // create a window to get keyboard events
    }

	std::cout << "Begin camera loop (press key to stop process)" << std::endl;

    while (true)
    {
        for (int camIndex = 0; camIndex < numDetectedCameras; camIndex++)
        {

            //if (newframe[camIndex])
			{
				if (current_frame[camIndex].size().area() > 0)
					cv::imshow(windowNames[camIndex], current_frame[camIndex]);
				//newframe[camIndex] = false;
			}

            
        }

        // quite program on keyboard input
		char c = cvWaitKey(1);
		if (c != -1)
			break;


    }

	std::cout << "\nCamera loop stopped.\n\n";

	std::cout << "Disconnecting cameras ...\n";

    stopCameras();

    cv::destroyAllWindows();

	std::cout << "done.\n\nRegular program exit.\n" << std::endl;

    return 0;
}

