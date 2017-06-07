#pragma once

#include <thread>
#include <chrono>
#include <mutex>

#include "Camera.h"

#include "stdafx.h"

#ifdef WIN32
#include "ps3eye.h"
#endif


#ifdef UNIX
// boost
#include <regex>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/regex.hpp>
#endif

namespace PS3EYECam
{

    enum class EOutputFormat
    {
        Bayer,
        BGR,
        RGB
    };


    class PS3Camera
    {

        int apiID;

    public :

        PS3Camera(std::string deviceName) :
            _deviceName(deviceName),
            _initialized(false)
        {

            apiID = cv::CAP_V4L2;      // choose API

        };

        ~PS3Camera()
        {
           release();

        };

        bool isInitialized() { return _initialized; }

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

        bool init()
        {
            // try to open device

            _deviceID = getDeviceID(_deviceName); // open camera


            // open selected camera using selected API

            _cap.open(apiID + _deviceID);

            if (_cap.isOpened())
            {
                _initialized = true;
                _cap.release();
            }
            else
                _initialized = false;

            return _initialized;
        }

        bool init(uint resX, uint resY, uint framerate, EOutputFormat format)
        {

            _width = resX;
            _height = resY;
            _framerate = framerate;

            if (format == EOutputFormat::Bayer)
                _numChannelPerPixel = 1;
            else
                _numChannelPerPixel = 3;


//            cap.set(CV_CAP_PROP_FPS, 30.0);
//            cap.set(CV_CAP_PROP_FRAME_WIDTH, 320);
//            cap.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

//            std::string v4cl2cmd = "v4l2-ctl -d "+d + " ";

//            std::string output;
//            LoggedSystem("", v4cl2cmd + "-c sharpness=0",output); // [0,63]
//            LoggedSystem("", v4cl2cmd + "-c auto_exposure=1",output); // 1 is manual exposure
//            LoggedSystem("", v4cl2cmd + "-c gain_automatic=0",output); // [0,1]
//            LoggedSystem("", v4cl2cmd + "-c gain=0",output); // [0,63]
//            LoggedSystem("", v4cl2cmd + "-c exposure=255",output); // [0,255]

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



            printf("init %d %d %d\n", _width, _height, _framerate);

            return true;
        }

        void release()
        {
            stop();
            _initialized = false;
        }

        void start()
        {

            LOGCON("camera %d start()\n", _deviceID);
            _cap.open(apiID + _deviceID);

            _cap.set(CV_CAP_PROP_FPS, _framerate);
            _cap.set(CV_CAP_PROP_FRAME_WIDTH, _width);
            _cap.set(CV_CAP_PROP_FRAME_HEIGHT, _height);

        }


        void stop()
        {
            LOGCON("camera %d stop()\n", _deviceID);
            _cap.release();
        }

        void getFrame(uint_fast8_t* frame)
        {

            if (_cap.isOpened())
            {
                //cv::Mat mat;
                _cap >> _latestFrame;
                frame = (uint8_t *)_latestFrame.ptr();
                //frame = _latestFrame.ptr();
            }
            else
            {
                std::cout << "camera is not opened !" << std::endl;
            }
        }

        void getFrame(cv::Mat &img)
        {

            if (_cap.isOpened())
            {
                //cv::Mat mat;
                _cap >> img;
            }
            else
            {
                std::cout << "camera is not opened !" << std::endl;
            }
        }



        uint8_t getExposure() { return _exposure;}
        uint8_t getGain() { return _gain;}
        uint8_t getContrast() { return _contrast;}
        uint8_t getBrightness() { return _brightness;}
        uint8_t getSharpness() { return _sharpness;}

        uint8_t getAutogain() { return _autogain;}
        uint8_t getAutoExposure() { return _autoexposure;}
        uint8_t getAutoWhiteBalance() { return _autowhitebalance;}
        uint8_t getFlipV() { return _flipV;}
        uint8_t getFlipH() { return _flipH;}

        void setExposure(uint8_t exposure) {
            _exposure = exposure;
        }

        void setGain(uint8_t gain) {
            _gain = gain;
        }

        void setContrast(uint8_t contrast) {
            _contrast = contrast;
        }

        void setBrightness(uint8_t brightness) {
            _brightness = brightness;
        }

        void setSharpness(uint8_t sharpness) {
            _sharpness = sharpness;
        }

        void setAutogain(uint8_t autogain) {
            _autogain = autogain;
        }

        void setAutoExposure(uint8_t autoexposure) {
            _autoexposure = autoexposure;
        }

        void setAutoWhiteBalance(uint8_t autowhitebalance) {
            _autowhitebalance = autowhitebalance;
        }

        void setFlipV(uint8_t flipv) {
            _flipV = flipv;
        }

        void setFlipH(uint8_t fliph) {
            _flipH = fliph;
        }

        uint getWidth() { return _width;}
        uint getHeight() { return _height;}
        uint getFrameRate() { return _framerate;}
        uint getOutputBytesPerPixel() { return _numChannelPerPixel;}

    private:

        cv::Mat _latestFrame;

        cv::VideoCapture    _cap;        // frame grabber
        int                 _deviceID;      // device id
        std::string         _deviceName;    // device name /dev/videoX

        bool                _initialized;

        uint    _width;
        uint    _height;

        uint    _framerate;

        uint    _numChannelPerPixel;

        uint8_t _exposure;
        uint8_t _gain;
        uint8_t _sharpness;
        uint8_t _contrast;
        uint8_t _brightness;
        uint8_t _autoexposure;
        uint8_t _autowhitebalance;
        uint8_t _autogain;
        uint8_t _flipV;
        uint8_t _flipH;

    };

    typedef PS3Camera* PS3EYERef;


    static void infoLogger(const std::string& line, std::stringstream &str)
    {
         //printf ("%s.\n",line.c_str());
         str << line << std::endl;
    }

    static int LoggedSystem(const std::string& prefix, const std::string& cmd, std::string &output)
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

    static int getSonyEyeDevices(std::vector<std::string> &sonyEyeDevices)
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
            printf("checking %s\n", devices[i].c_str());

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
                    if (boost::algorithm::ifind_first(devicename,"id_serial="))
                    {
                        printf("%s\n", s.c_str());
                        if (boost::algorithm::ifind_first(devicename,"id_serial=omnivision"))
                        {
                            // add sony eye to to device list
                            sonyEyeDevices.push_back(canonicalDevice);
                            break;
                        }
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


    static std::vector<std::string> deviceNames;
    static int numSonyEyeCameras = 0;
    static std::vector<PS3Camera> sonyEyeCams;

    static bool setupDevices()
    {
        numSonyEyeCameras = getSonyEyeDevices(deviceNames);

        if (numSonyEyeCameras == 0)
        {
            printf("No sony eye camera found! Exiting...\n");
        }

        // create sony eye camera for each camera
        for (int i = 0; i < numSonyEyeCameras; i++)
        {
            PS3Camera cam(deviceNames[i]);
            if (cam.init())
                sonyEyeCams.push_back(cam);
            else
            {
                printf("Camera %s could not be initialized.\n", deviceNames[i]);
            }
        }

        if (sonyEyeCams.size() == numSonyEyeCameras)
            return true;

        else
        {
            // release cameras due to error
            for (auto c : sonyEyeCams)
                c.release();
            sonyEyeCams.clear();
        }

        return false;

    }

    static std::vector<PS3EYERef> getDevices()
    {
        std::vector<PS3EYERef> devices;
        for (int c = 0; c < sonyEyeCams.size(); c++ )
        {
            PS3EYERef camPtr = &sonyEyeCams[c];
            devices.push_back(camPtr);
        }
        return devices;
    }


}

class MyClass
{

public:
	/* Explicitly using the default constructor to
	* underline the fact that it does get called */
	MyClass() : the_thread() {}
	~MyClass() {
		stop_thread = true;
		if (the_thread.joinable()) the_thread.join();
	}

	void Start() {
		// This will start the thread. Notice move semantics!
		the_thread = std::thread(&MyClass::ThreadMain, this);
	}

	void Stop() {
		stop_thread = true;
		if (the_thread.joinable()) the_thread.join();
	}

private:
	std::thread the_thread;
	
	bool stop_thread = false; // Super simple thread stopping.

	void ThreadMain()
	{
		while (!stop_thread)
		{
			std::cout << "thread iteration" << std::endl;
			// Do something useful, e.g:
			std::this_thread::sleep_for(std::chrono::seconds(1));
		}
	}

	/****/
};


class ThreadCamera : public Camera
{

public:
	/* Explicitly using the default constructor to
	* underline the fact that it does get called */
	ThreadCamera() : the_thread()
	{

	}
	~ThreadCamera() {
		deinitialize();
	}

	// implement camera interface

	bool initialize()
	{
		// default values
        _deviceID           = 0;
        _resolution.x       = 640;
        _resolution.y       = 480;
        _framerate          = 60;
        _numColorChannels   = 3;

		return true;
	};

	bool initialize(
		unsigned int deviceID,
		unsigned int camResolutionWidth,
		unsigned int camResolutionHeight,
		unsigned int numColorChannels,
		unsigned int framerate)
	{

        _deviceID           = deviceID;
        _resolution.x       = camResolutionWidth;
        _resolution.y       = camResolutionHeight;
        _framerate          = framerate;
        _numColorChannels   = numColorChannels;

		return true;
	}

	void deinitialize() {
	
		stopCapture();

		_isInitialized = false;
	
	};

	/*
	Apply callback function that will be called on the capturing thread for every frame.
	*/
	void setCallback(void(*callbackFunction)(cv::Mat frame, void *userdata), void *userData) {
		processFrame = callbackFunction;
		userdata = userData;
	}

	bool startCapture()
	{
		// This will start the thread. Notice move semantics!
        the_thread = std::thread(&ThreadCamera::ThreadMain, this);
		
		return true;
	};
	bool stopCapture()
	{
		stop_thread = true;
		if (the_thread.joinable()) the_thread.join();

		return true;
	};

	// copy latest frame thread-safe
	void receiveFrameCopy(cv::Mat &frame)
	{
		
		g_pages_mutex.lock();

		_latestCamFrame.copyTo(frame);

		g_pages_mutex.unlock();
	};

	unsigned int getCameraWidth() { return _resolution.x; };
	unsigned int getCameraHeight() { return _resolution.y; };
	unsigned int getFramerate() { return _framerate; }
	unsigned int getNumColorChannels() { return _numColorChannels; }

private:
	std::thread the_thread;

	std::mutex g_pages_mutex;

	/*
	Callback function to process each captured frame.
	*/
	void(*processFrame)(cv::Mat frame, void *userdata) = NULL;
	void *userdata;

	bool stop_thread = false; // Super simple thread stopping.

	bool initCamera()
	{

        printf("initCamera()\n");

		// list out the devices

        #ifdef WIN32
        using namespace ps3eye;
		std::vector<PS3EYECam::PS3EYERef> devices(PS3EYECam::getDevices());
        #endif

        #ifdef UNIX
        std::vector<PS3EYECam::PS3EYERef> devices = PS3EYECam::getDevices();
        #endif

		LOGCON("Found %d cameras.\n", (int)devices.size());

		bool initializationResult = false;

		if (devices.size() > 0)
		{
			_cameraPtr = devices.at(_deviceID);



			if (_numColorChannels == 3)
                initializationResult = _cameraPtr->init(_resolution.x, _resolution.y, _framerate, PS3EYECam::EOutputFormat::BGR);
			else
                initializationResult = _cameraPtr->init(_resolution.x, _resolution.y, _framerate, PS3EYECam::EOutputFormat::Bayer);

			if (initializationResult)
			{

                LOGCON("Initialization of PS3EyeCam (id %d) successful !\n", _deviceID);

                _resolution.x       = _cameraPtr->getWidth();
                _resolution.y       = _cameraPtr->getHeight();
                _framerate          = _cameraPtr->getFrameRate();
                _numColorChannels   = _cameraPtr->getOutputBytesPerPixel();

    \
                LOGCON("Allocating memory for frame dimension [%d, %d, %d]\n", _resolution.x, _resolution.y, _numColorChannels);


				if (_numColorChannels == 3)
					_latestCamFrame = cv::Mat(_resolution.y, _resolution.x, CV_8UC3);
				else
					_latestCamFrame = cv::Mat(_resolution.y, _resolution.x, CV_8UC1);

			}
			else {
                LOGERROR("Initialization of PS3EyeCam (id %d) failed !!\n", _deviceID);
				return false;
			}
		}

		frame_bgr = new uint8_t[_resolution.x * _resolution.y * _numColorChannels];

		_isInitialized = _cameraPtr->isInitialized();

        if (_isInitialized)
            printf("PS3EyeCam (id %d) is initialzed.\n", _deviceID);
        else
            printf("PS3EyeCam (id %d) could not be initialzed.\n", _deviceID);

		return _isInitialized;

	}

	void resetFrameCounter()
	{
		// reset frame counter
		_currentfps = 0;
		_fpsCount = 0;
		_lastTime = time(0);
	}
	
	void updateFrameCounter()
	{
		// current FPS computation
		_fpsCount++;
		if (time(0) > _lastTime) {
			_currentfps = _fpsCount;
			_fpsCount = 0;
			_lastTime = time(0);
			printf("cam %d fps = %d\n", _deviceID, _currentfps);
		}
	}

	void ThreadMain()
	{

        printf("ThreadMain\n");

		// initialize camera
		if (!initCamera())
			return;
		
		_cameraPtr->start();
		
		while (!stop_thread)
		{

            //_cameraPtr->getFrame(frame_bgr);

			g_pages_mutex.lock();
			{

                //unsigned char *input = (unsigned char*)(_latestCamFrame.data);
                //memcpy(input, frame_bgr, sizeof(uint8_t) * _resolution.x * _resolution.y * _numColorChannels);

                _cameraPtr->getFrame(_latestCamFrame);

				// if callback function is set, return image to the function
				if (processFrame)
				{
					processFrame(_latestCamFrame, userdata);
				}

			}
			g_pages_mutex.unlock();


			updateFrameCounter();

		}

		_cameraPtr->stop();

	}

	// camera members
	uint8_t	*frame_bgr;
	cv::Mat _latestCamFrame;

#ifdef WIN32
	ps3eye::PS3EYECam::PS3EYERef _cameraPtr;
#endif
#ifdef UNIX
    PS3EYECam::PS3EYERef _cameraPtr;
#endif

	unsigned int	_deviceID;

	cv::Point2i		_resolution;
	unsigned int	_framerate;
	unsigned int	_numColorChannels;

	int _currentfps, _fpsCount = 0;
	time_t _lastTime = time(0);

	bool			_isInitialized = false;

public:

	int				_exposure = 50;
	int				_gain = 0;
	int				_brightness = 0;
	int				_contrast = 0;
	int				_sharpness = 0;

	bool			_autogain = false;
	bool			_autowhitebalance = false;

	bool			_flipHorizontally = false;
	bool			_flipVertically = false;


    void updateCameraSettings() {

		if (!_isInitialized)
			return;

        uint8_t exposure        = _cameraPtr->getExposure();
        uint8_t gain            = _cameraPtr->getGain();
        uint8_t brightness      = _cameraPtr->getBrightness();
        uint8_t contrast        = _cameraPtr->getContrast();
        uint8_t sharpness       = _cameraPtr->getSharpness();

        bool flipVertical       = _cameraPtr->getFlipV();
        bool flipHorizontal     = _cameraPtr->getFlipH();

        bool autoGain           = _cameraPtr->getAutogain();
        bool autoWhiteBalance   = _cameraPtr->getAutoWhiteBalance();

		if (exposure != _exposure) {
			_cameraPtr->setExposure(uint8_t(_exposure));
			//LOGCON("Setting expsure to %d\n", uint8_t(_exposure));
		}
		if (gain != _gain) {
			_cameraPtr->setGain(uint8_t(_gain));
			//LOGCON("Setting gain to %d\n", uint8_t(_gain));
		}
		if (brightness != _brightness) {
			_cameraPtr->setBrightness(uint8_t(_brightness));
			LOGCON("Setting brightness to %d\n", uint8_t(_gain));
			LOGWARNING("brightness filter not implemented yet!\n");
		}
		if (contrast != _contrast) {
			_cameraPtr->setContrast(uint8_t(_contrast));
			LOGCON("Setting contrast to %d\n", uint8_t(_gain));
			LOGWARNING("contrast filter not implemented yet!\n");
		}
		if (sharpness != _sharpness) {
			_cameraPtr->setSharpness(uint8_t(_sharpness));
			LOGCON("Setting sharpness to %d\n", uint8_t(_gain));
			LOGWARNING("sharpness filter not implemented yet!\n");
		}

		if (autoGain != _autogain) {
			_cameraPtr->setAutogain(_autogain);
			LOGCON("Setting autogain to %d\n", _autogain);
		}

		if (autoWhiteBalance != _autowhitebalance) {
			_cameraPtr->setAutoWhiteBalance(_autowhitebalance);
			LOGCON("Setting autoWhiteBalance to %d\n", _autowhitebalance);
		}

	}


	/****/
};
