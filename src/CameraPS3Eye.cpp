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

	return initialize(640, 480, 3, 60);		// 60 Hz VGA
	// return initialize(320, 240, 3, 120);		// 120 Hz QVGA
}

bool CameraPS3Eye::initialize(
	unsigned int camResolutionWidth = 640,
	unsigned int camResolutionHeight = 320,
	unsigned int numColorChannels = 3,
	unsigned int framerate = 5)
{

	_resolution.x = camResolutionWidth;
	_resolution.y = camResolutionHeight;
	_framerate = framerate;
	_numColorChannels = numColorChannels;

	using namespace ps3eye;

	// list out the devices
	std::vector<PS3EYECam::PS3EYERef> devices(PS3EYECam::getDevices());
	LOGCON("Found %d cameras.\n", (int)devices.size());
	
	_cameraPtr = 0;

	bool initializationResult = false;

	if (devices.size() > 0)
	{
		_cameraPtr = devices.at(0);
		initializationResult = _cameraPtr->init(_resolution.x, _resolution.y, _framerate);

		

		if (initializationResult)
		{

			LOGCON("Initialization of PS3 Eye Cam successful !\n");

			startCapture();


			_resolution.x = _cameraPtr->getWidth();
			_resolution.y = _cameraPtr->getHeight();
			_framerate = _cameraPtr->getFrameRate();
			_numColorChannels = _cameraPtr->getOutputBytesPerPixel();

			// allocate memory for output frame
			frame_bgr = new uint8_t[_resolution.x * _resolution.y * _numColorChannels];

			_latestCamFrame = cv::Mat(_resolution.y, _resolution.x, CV_8UC3);

		}
		else {
			LOGERROR("Initialization of PS3 Eye Cam faild !!\n");
		}
	}

	return initializationResult;

}


cv::Mat CameraPS3Eye::receiveFrame()
{

	if (_isPaused)
		return _latestCamFrame;

	cv::Mat cameraFrame;
	if (_cameraPtr)
	{
		_cameraPtr->getFrame(frame_bgr);

		// copy to OpenCV image
		unsigned char *input = (unsigned char*)(_latestCamFrame.data);
		memcpy(input, frame_bgr, sizeof(uint8_t) * _resolution.x * _resolution.y * 3);

		if (_flipHorizontally && _flipVertically)
			 cv::flip(_latestCamFrame, _latestCamFrame, -1);
		else if (_flipHorizontally)
			cv::flip(_latestCamFrame, _latestCamFrame, 0);
		else if (_flipVertically)
			cv::flip(_latestCamFrame, _latestCamFrame, 1);

		//_latestCamFrame.data = frame_bgr;
		
		// DEBUG : show frame using OpenCV function
		//cv::imshow("camera", _latestCamFrame);
		//cv::waitKey(1);
		
		cameraFrame = _latestCamFrame;
	}

	return cameraFrame;
	
}

void CameraPS3Eye::deinitialize()
{

	if (_cameraPtr)
		stopCapture();

}

void CameraPS3Eye::startCapture()
{
	_cameraPtr->start();
}

void CameraPS3Eye::stopCapture()
{
	if (_cameraPtr)
		_cameraPtr->stop(); // throwing error in debug
}


void CameraPS3Eye::updateCameraSettings() {

	uint8_t exposure	= _cameraPtr->getExposure();
	uint8_t gain		= _cameraPtr->getGain();
	uint8_t brightness	= _cameraPtr->getBrightness();
	uint8_t contrast	= _cameraPtr->getContrast();
	uint8_t sharpness	= _cameraPtr->getSharpness();

	bool flipVertical	= _cameraPtr->getFlipV();
	bool flipHorizontal = _cameraPtr->getFlipH();

	float multiplier = 256.f;

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

	//if (flipVertical != _flipVertically || flipHorizontal != _flipHorizontally) {
	//	_cameraPtr->setFlip(_flipHorizontally, _flipVertically);
	//}



}
