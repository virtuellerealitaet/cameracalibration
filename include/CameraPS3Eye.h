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

#pragma once

#include "Camera.h"
#include "ps3eye.h"

class cv::Mat;

class CameraPS3Eye : public Camera
{

public:

	CameraPS3Eye()
	{
	}

	~CameraPS3Eye()
	{
		deinitialize();
	}

	bool initialize();

	bool initialize(unsigned int camResolutionWidth,
		unsigned int camResolutionHeight,
		unsigned int numColorChannels,
		unsigned int framerate);
	void deinitialize();

	void startCapture();
	void stopCapture();

	unsigned int getCameraWidth() { return _resolution.x;}
	unsigned int getCameraHeight() { return _resolution.y; }
	unsigned int getFramerate() { return _framerate; }
	unsigned int getNumColorChannels() { return _numColorChannels; }
	

	// CB function for new frame
	cv::Mat receiveFrame();
	
	ps3eye::PS3EYECam::PS3EYERef getCameraPtr() { return _cameraPtr; }

	void updateCameraSettings();

	int		_exposure	= 50;
	int		_gain		= 0;
	int		_brightness = 0;
	int		_contrast	= 0;
	int		_sharpness	= 0;
	
	bool	_flipHorizontally = false;
	bool	_flipVertically = false;

private:
	ps3eye::PS3EYECam::PS3EYERef _cameraPtr;
	
	cv::Point2i		_resolution;
	unsigned int	_framerate;
	unsigned int	_numColorChannels;

	uint8_t			*frame_bgr;
	cv::Mat			_latestCamFrame;



};
