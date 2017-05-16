/*****************************************************************************
* Application :		Camera Calibration Application
*					using OpenCV3 (http://opencv.org/)
*					and PS3EYEDriver C API Interface (by Thomas Perl)
*
* Author      :		Michael Stengel <virtuellerealitaet@gmail.com>
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    1. Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*
*    2. Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
**/

#include "stdafx.h"

#include "CameraPS3Eye.h"

bool CameraPS3Eye::initialize() {

	return initialize(640, 480, 3, 60, 0);			// 60 Hz VGA
	// return initialize(320, 240, 3, 120, 0);		// 120 Hz QVGA
}

bool CameraPS3Eye::initialize(
	unsigned int camResolutionWidth = 640,
	unsigned int camResolutionHeight = 320,
	unsigned int numColorChannels = 3,
	unsigned int framerate = 5,
	unsigned int deviceID = 0)
{

	_resolution.x = camResolutionWidth;
	_resolution.y = camResolutionHeight;
	_framerate = framerate;
	_numColorChannels = numColorChannels;
	_deviceID = deviceID;


	return true;

}


cv::Mat CameraPS3Eye::receiveFrame()
{

	cv::Mat cameraFrame;

	/*

	EnterCriticalSection(mutex);

	if (_isPaused)
		return _latestCamFrame;

	cv::Mat cameraFrame;
	if (_cameraPtr)
	{
		// copy to OpenCV image
		unsigned char *input = (unsigned char*)(_latestCamFrame.data);
		memcpy(input, frame_bgr, sizeof(uint8_t) * _resolution.x * _resolution.y * _numColorChannels);

		if (_flipHorizontally && _flipVertically)
			 cv::flip(_latestCamFrame, _latestCamFrame, -1);
		else if (_flipHorizontally)
			cv::flip(_latestCamFrame, _latestCamFrame, 0);
		else if (_flipVertically)
			cv::flip(_latestCamFrame, _latestCamFrame, 1);
		
		cameraFrame = _latestCamFrame;
	}

	LeaveCriticalSection(mutex);

	*/

	return cameraFrame;


	
}

void CameraPS3Eye::Run()
{

	_currentfps = 0;
	_fpsCount = 0;
	_lastTime = time(0);

	using namespace ps3eye;

	// list out the devices
	std::vector<PS3EYECam::PS3EYERef> devices(PS3EYECam::getDevices());
	LOGCON("Found %d cameras.\n", (int)devices.size());



	bool initializationResult = false;

	if (devices.size() > 0)
	{
		_cameraPtr = devices.at(_deviceID);
		if (_numColorChannels == 3)
			initializationResult = _cameraPtr->init(_resolution.x, _resolution.y, _framerate, ps3eye::PS3EYECam::EOutputFormat::BGR);
		else
			initializationResult = _cameraPtr->init(_resolution.x, _resolution.y, _framerate, ps3eye::PS3EYECam::EOutputFormat::Bayer);

		if (initializationResult)
		{

			LOGCON("Initialization of PS3 Eye Cam successful !\n");

			//startCapture();

			_resolution.x = _cameraPtr->getWidth();
			_resolution.y = _cameraPtr->getHeight();
			_framerate = _cameraPtr->getFrameRate();
			_numColorChannels = _cameraPtr->getOutputBytesPerPixel();

			// allocate memory for output frame
			//frame_bgr = new uint8_t[_resolution.x * _resolution.y * _numColorChannels];

			if (_numColorChannels == 3)
				_latestCamFrame = cv::Mat(_resolution.y, _resolution.x, CV_8UC3);
			else
				_latestCamFrame = cv::Mat(_resolution.y, _resolution.x, CV_8UC1);

		}
		else {
			LOGERROR("Initialization of PS3 Eye Cam faild !!\n");
		}
	}

	_cameraPtr->start();


	frame_bgr = new uint8_t[_resolution.x * _resolution.y * _numColorChannels];



	// image capturing loop
	while (_running && _cameraPtr)
	{
		
		//if (_cameraPtr->isInitialized())
		_cameraPtr->getFrame(frame_bgr);
		
		
		//EnterCriticalSection(mutex);
				

		// current FPS computation
		_fpsCount++;
		if (time(0) > _lastTime) {
			_currentfps = _fpsCount;
			_fpsCount = 0;
			_lastTime = time(0);
			printf("fps = %d\n", _currentfps);
		}

		

		// if callback function is set, return image to the function
		if (processFrame)
		{

			// copy camera frame to OpenCV image
			cv::Mat frame;
			frame = cv::Mat(_resolution.y, _resolution.x, CV_8UC3);

			unsigned char *input = (unsigned char*)(frame.data);
			memcpy(input, frame_bgr, sizeof(uint8_t) * _resolution.x * _resolution.y * _numColorChannels);

			if (_flipHorizontally && _flipVertically)
				cv::flip(frame, frame, -1);
			else if (_flipHorizontally)
				cv::flip(frame, frame, 0);
			else if (_flipVertically)
				cv::flip(frame, frame, 1);


			processFrame(frame, userdata);

		}

		//Sleep(3);

		//LeaveCriticalSection(mutex);

	}

	_cameraPtr->stop(); // throwing error in debug

}


void CameraPS3Eye::deinitialize()
{

	if (_cameraPtr)
		stopCapture();

}

bool CameraPS3Eye::startCapture()
{

	_running = true;
	
	// MUTEX CONDITION
	InitializeCriticalSection(mutex);

	// Start image capture thread
	DWORD threadId = 0;
	_hThread = CreateThread(NULL, 0, &CameraPS3Eye::CaptureThread, this, 0, &threadId);

	LOGDEBUG("Creating camera capture thread (id = %d)\n", threadId);

	// set processor to run the thread on
	DWORD_PTR i = DWORD_PTR(_core);
	DWORD_PTR m_mask = 1 << i;

	BOOL success = SetThreadAffinityMask(_hThread, m_mask);
	if (success == 0)
	{
		LOGWARNING("camera capture thread : Setting the Thread Affinity for Main Process could not be done\n");
	}
	else {
		LOGDEBUG("camera capture thread : Creating Thread %d (0x%08x) Assigning to CPU 0x%08x\r\n", i, (LONG_PTR)threadId, m_mask); //just so we can see whats going on as it goes on (if your not using unicode, you should use printf here and not wprintf.
	}

	if (_hThread == NULL)
	{
		LOGERROR("Could not create camera capture thread\n");
		return false;
	}

	

	return true;
}

bool CameraPS3Eye::stopCapture()
{

	if (!_running)
		return false;

	_running = false;
	WaitForSingleObject(_hThread, 2000);

	while (_cameraPtr && _cameraPtr->isStreaming())
	{
		_cameraPtr->stop(); // throwing error in debug
		Sleep(100);
		LOGDEBUG("Waiting for camera to stop...\n");
	}
	LOGDEBUG("camera stopped.\n");
	
	return true;
}

void CameraPS3Eye::updateCameraSettings() {

	uint8_t exposure	= _cameraPtr->getExposure();
	uint8_t gain		= _cameraPtr->getGain();
	uint8_t brightness	= _cameraPtr->getBrightness();
	uint8_t contrast	= _cameraPtr->getContrast();
	uint8_t sharpness	= _cameraPtr->getSharpness();

	bool flipVertical	= _cameraPtr->getFlipV();
	bool flipHorizontal = _cameraPtr->getFlipH();

	bool autoGain			= _cameraPtr->getAutogain();
	bool autoWhiteBalance	= _cameraPtr->getAutoWhiteBalance();

	if (exposure != _exposure)		{
		_cameraPtr->setExposure(uint8_t(_exposure));
		//LOGCON("Setting expsure to %d\n", uint8_t(_exposure));
	}
	if (gain != _gain)				{
		_cameraPtr->setGain(uint8_t(_gain));
		//LOGCON("Setting gain to %d\n", uint8_t(_gain));
	}
	if (brightness != _brightness)	{
		_cameraPtr->setBrightness(uint8_t(_brightness));
		LOGCON("Setting brightness to %d\n", uint8_t(_gain));
		LOGWARNING("brightness filter not implemented yet!\n");
	}
	if (contrast != _contrast)		{
		_cameraPtr->setContrast(uint8_t(_contrast));
		LOGCON("Setting contrast to %d\n", uint8_t(_gain));
		LOGWARNING("contrast filter not implemented yet!\n");
	}
	if (sharpness != _sharpness)	{
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
