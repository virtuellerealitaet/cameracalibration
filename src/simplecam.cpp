
#include "stdafx.h"

// qt includes
#include <QApplication>


#include "ThreadCamera.h"


const int NUM_CAMERAS = 1;

static ThreadCamera *VideoCapCam[NUM_CAMERAS];
static bool newframe[NUM_CAMERAS];
static cv::Mat current_frame[NUM_CAMERAS];
static cv::Mat roi[NUM_CAMERAS];
static cv::Mat combined;

static void cameracallback(cv::Mat frame, void *userdata)
{
    //int camIndex = (int)userdata;
    int camIndex = *((int*)(&userdata));

    std::cout << "new frame for index " << camIndex << std::endl;

    frame.copyTo(current_frame[camIndex]);
    newframe[camIndex] = true;
}

bool startCameras()
{


    if (!PS3EYECam::setupDevices())
    {
        printf("Initialization of Sony Eye Cam(s) failed. Exiting..\n");
        return 0;
    }


    std::vector<PS3EYECam::PS3EYERef> devices = PS3EYECam::getDevices();



    // load an image

    int framecounter = 0;

    int numCameras = devices.size();
    if (numCameras > 0)
    {



        for (int camIdx = 0; camIdx < numCameras; camIdx++)
        {
            VideoCapCam[camIdx] = new ThreadCamera();

            VideoCapCam[camIdx]->setCallback(cameracallback, (void*)camIdx);

            bool success = VideoCapCam[camIdx]->initialize(camIdx, 640, 480, 3, 30);
            if (!success)
                return false;

        }

    }



    for (int camIdx = 0; camIdx < numCameras; camIdx++)
    {
        printf("starting camera %d\n", camIdx);

        VideoCapCam[camIdx]->startCapture();

    }


    return true;

}


bool stopCameras()
{

    int numCameras = NUM_CAMERAS;

    for (int camIdx = 0; camIdx < numCameras; camIdx++)
    {
        VideoCapCam[camIdx]->stopCapture();

        printf("stopping camera %d\n", camIdx);
    }

    for (int camIdx = 0; camIdx < numCameras; camIdx++)
    {
        delete VideoCapCam[camIdx];

        printf("deleting camera %d\n", camIdx);
    }

    return true;
}


void mouseCB(int event, int x, int y, int flags, void*param)
{

}


int main(int argc, char *argv[])
{

    startCameras();

    std::string windowName = "camera";
    cv::namedWindow(windowName); // create a window to get keyboard events
    //cv::setMouseCallback("Camera", mouseCB, 0);

    while (true)
    {

        printf("main loop...\n");
        //std::this_thread::sleep_for(std::chrono::milliseconds(100));

        // quite program on keyboard input
        char c = cvWaitKey(100);
        if (c != -1)
          break;

        if (newframe[0])
        {

            cv::imshow(windowName,current_frame[0]);
            cv::waitKey(1);

        }


    }

    stopCameras();

    cv::destroyAllWindows();

    return 0;
}

