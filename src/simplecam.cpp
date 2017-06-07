
#include "stdafx.h"

// qt includes
#include <QApplication>

#include "ThreadCamera.h"

#if 0

// opencv includes
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

// system
#include <iostream>   // std::cout
#include <chrono>
#include <thread>

// boost
#include <regex>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>


void infoLogger(const std::string& line, std::stringstream &str)
{
     //printf ("%s.\n",line.c_str());
     str << line << std::endl;
}

int LoggedSystem(const std::string& prefix, const std::string& cmd, std::string &output)
{
    std::stringstream str;

    printf ("%s.\n",cmd.c_str());

    FILE* fpipe = popen(cmd.c_str(), "r");
    if (fpipe == NULL)
        throw std::runtime_error(std::string("Can't run ") + cmd);
    char* lineptr;
    size_t n;
    ssize_t s;
    do {
        lineptr = NULL;
        s = getline(&lineptr, &n, fpipe);
        if (s > 0 && lineptr != NULL) {
            if (lineptr[s - 1] == '\n')
                lineptr[--s  ] = 0;
            if (lineptr[s - 1] == '\r')
                lineptr[--s  ] = 0;
            infoLogger(prefix + lineptr, str);
        }
        if (lineptr != NULL)
            free(lineptr);
    } while (s > 0);
    int status = pclose(fpipe);

    //infoLogger(std::to_string(status), str);

    output = str.str();

    return status;
}

int countNumSonyEye(std::vector<std::string> &sonyEyeDevices)
{

    // get all devices registered by video4linux
    std::string v4lDevices;
    std::string basePath = "/dev/v4l/by-path/";
    LoggedSystem ("","ls "+basePath, v4lDevices);
    std::vector<std::string> devices;
    boost::split(devices,v4lDevices,boost::is_any_of("\n"));


    // filter for sony eye cam
    for (int i = 0 ; i < (int)devices.size(); i++)
    {
        //printf("checking %s\n", devices[i].c_str());

        if (devices[i].length() > 0)
        {
            // get device name from v4l path
            boost::filesystem::path path( basePath + devices[i]);
            std::string canonicalDevice = boost::filesystem::canonical(path).string();

            // query camera information using udevadm
            std::string cmd("udevadm info --query=all --name=");
            cmd.append(canonicalDevice);
            std::string deviceInfo;
            LoggedSystem ("", cmd, deviceInfo);

            // split return info
            std::vector<std::string> deviceInfoLines;
            boost::split(deviceInfoLines,deviceInfo,boost::is_any_of("\n"));

            // check line by line for ID_SERIAL info containing specific sony eye title
            for (auto s : deviceInfoLines)
            {
                // check id serial for omnivision camera (sony eye 3)
                std::string devicename = boost::algorithm::to_lower_copy(s);
                if (boost::algorithm::ifind_first(devicename,"id_serial=omnivision"))
                {
                    // add sony eye to to device list
                    sonyEyeDevices.push_back(canonicalDevice);
                    break;
                }
            }
        }
    }

    int deviceID = 0;
    for (auto s : sonyEyeDevices)
    {
        printf("sony eye cam %d : %s\n", deviceID, s.c_str());
        deviceID++;
    }

    return sonyEyeDevices.size();

}

int getDeviceID(std::string resource)
{

  const char * pattern = "\\d+";

  boost::regex re(pattern);

  boost::sregex_iterator it(resource.begin(), resource.end(), re);
  boost::sregex_iterator end;

  for( ; it != end; ++it)
  {
      std::cout<< it->str() <<"\n";
      return atoi(it->str().c_str());

  }

  return 0;

}

#endif

std::vector<ThreadCamera*> VideoCapCam;

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

        VideoCapCam.clear();

        for (int c = 0; c < numCameras; c++)
        {
            ThreadCamera *cam = new ThreadCamera();
            bool success = cam->initialize(c, 320, 240,3, 30);
            if (!success)
                return false;
            else
                VideoCapCam.push_back(cam);
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

    int numCameras = VideoCapCam.size();

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

    VideoCapCam.clear();



    return true;
}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

#if 0

    std::vector<std::string> devices;

    if (countNumSonyEye(devices) == 0){
        printf("No sony eye camera found! Exiting...\n");
        return 0;
    }

    // open each camera and grab frames
    std::vector<cv::VideoCapture> cameras;
    for (auto d : devices)
    {
        cv::VideoCapture cap;

        // select V4L2 API backend
        int deviceID = getDeviceID(d);             // 0 = open default camera
        int apiID = cv::CAP_V4L2;      // 0 = autodetect default API

        // open selected camera using selected API
        cap.open(deviceID + apiID);

        std::this_thread::sleep_for (std::chrono::seconds(1));

        cap.set(CV_CAP_PROP_FPS, 30.0);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

       std::string v4cl2cmd = "v4l2-ctl -d "+d + " ";

        std::string output;
        LoggedSystem("", v4cl2cmd + "-c sharpness=0",output); // [0,63]
        LoggedSystem("", v4cl2cmd + "-c auto_exposure=1",output); // 1 is manual exposure
        LoggedSystem("", v4cl2cmd + "-c gain_automatic=0",output); // [0,1]
        LoggedSystem("", v4cl2cmd + "-c gain=0",output); // [0,63]
        LoggedSystem("", v4cl2cmd + "-c exposure=255",output); // [0,255]

        //std::this_thread::sleep_for (std::chrono::seconds(1));



        //cap.set(CV_CAP_PROP_AUTO_EXPOSURE, 0);

        //cap.set(CV_CAP_PROP_GAIN, 0);
        //cap.set(CV_CAP_PROP_SHARPNESS, 0);

        //cap.set(CV_CAP_PROP_EXPOSURE, 4);

        // Get fixed Cam Properties

//            double Brightness = cap.get(CV_CAP_PROP_BRIGHTNESS);
//            double Contrast   = cap.get(CV_CAP_PROP_CONTRAST );
//            double Saturation = cap.get(CV_CAP_PROP_SATURATION);
//            cap.set(CV_CAP_PROP_GAIN, 0.0);
//            double Gain       = cap.get(CV_CAP_PROP_GAIN);
//            double Exposure   = cap.get(CV_CAP_PROP_AUTO_EXPOSURE );


//            // Display Them

//            std::cout<<"===================================="<<std::endl;
//            std::cout<<"Default Brightness -------> "<<Brightness<<std::endl;
//            std::cout<<"Default Contrast----------> "<<Contrast<<std::endl;
//            std::cout<<"Default Saturation--------> "<<Saturation<<std::endl;
//            std::cout<<"Default Gain--------------> "<<Gain<<std::endl;
//            std::cout<<"Default exp --------------> "<<Exposure<<std::endl;
//            std::cout<<"===================================="<<std::endl;


//            cap.set(CV_CAP_PROP_EXPOSURE,-100);


        if (!cap.isOpened())
        {
            std::cout << "could not open cam" << std::endl;
            return -1;
        }
        else
            cameras.push_back(cap);
    }

#endif

    startCameras();

    stopCameras();


#if 0

    int numCameras = devices.size();
    if (numCameras > 0)
    {
        for (;;)
        {

            for (int c = 0; c < numCameras; c++)
            {

                cv::Mat img;
                devices[c] >> img;

                // display image
                std::string camName = "sonyEye";
                camName.append(std::to_string(c));
                cv::imshow(camName, img);


                //framecounter++;
                //if (framecounter > 1000)
                //    break;

            }
            cv::waitKey(1);

        }
    }

    #endif



    return 0;
}

