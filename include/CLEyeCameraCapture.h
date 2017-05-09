#pragma once

#ifndef _WIN32_WINNT            // Specifies that the minimum required platform is Windows Vista.
#define _WIN32_WINNT 0x0600     // Change this to the appropriate value to target other versions of Windows.
#endif

#define NOMINMAX

// SYSTEM

#include <conio.h>
#include <stdio.h>
#include <tchar.h>
#include <windows.h>
#include <time.h>
#include <iostream>

// CL EYE SDK
#include "CLEyeMulticam.h"

// OPENCV
#include <opencv2\opencv.hpp>

#define  LOGCON(...)  {	printf(__VA_ARGS__);}
#define  LOGDEBUG(...)  {	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); SetConsoleTextAttribute(hConsole, 10); printf(__VA_ARGS__);SetConsoleTextAttribute(hConsole, 7);}
#define  LOGWARNING(...)  {	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); SetConsoleTextAttribute(hConsole, 14); printf("WARNING : "); printf(__VA_ARGS__);SetConsoleTextAttribute(hConsole, 7);}
#define  LOGERROR(...)  {	HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE); SetConsoleTextAttribute(hConsole, 12); printf("ERROR : "); printf(__VA_ARGS__);SetConsoleTextAttribute(hConsole, 7);}
#define MSGBOXERROR(...) { MessageBox(Window::theInstance()->_hWND, __VA_ARGS__, "ERROR", MB_ICONERROR);}
#define MSGBOXWARNING(...) { MessageBox(Window::theInstance()->_hWND, __VA_ARGS__, "WARNING", MB_ICONWARNING);}
#define MSGBOXINFO(...) { MessageBox(Window::theInstance()->_hWND, __VA_ARGS__, "INFO", MB_ICONINFORMATION);}

using namespace cv;

static LARGE_INTEGER _frequencyPT;

static inline PVOID ProfileMSStart()
{
	PLARGE_INTEGER start = new LARGE_INTEGER;
	QueryPerformanceFrequency(&_frequencyPT);
	QueryPerformanceCounter(start);
	return (PVOID)start;
}

static inline double ProfileMSEnd(PVOID p)
{
	LARGE_INTEGER stop, diff;
	PLARGE_INTEGER start = (PLARGE_INTEGER)p;
	QueryPerformanceCounter(&stop);
	diff.QuadPart = stop.QuadPart - start->QuadPart;
	double timeMs = 1000.0 * ((double)diff.QuadPart / (double)_frequencyPT.QuadPart);
	delete start;
	return timeMs;
}

static double GetRandomNormalized()
{
	return (double)(rand() - (RAND_MAX >> 1)) / (double)(RAND_MAX >> 1);
}

static DWORD_PTR GetNumCPUs() {

		SYSTEM_INFO m_si = { 0, };

		GetSystemInfo(&m_si);

		return (DWORD_PTR)m_si.dwNumberOfProcessors;

}

inline std::vector<std::string> split(const std::string &s, char delim) {
	std::stringstream ss(s);
	std::string item;
	std::vector<std::string> elems;
	while (std::getline(ss, item, delim)) {
		elems.push_back(item);
		// elems.push_back(std::move(item)); // if C++11 (based on comment from @mchiasson)
	}
	return elems;
}

inline std::string GUIDToString(GUID guid) {
	char guid_str[1024];
	sprintf(guid_str, "%08x-%04x-%04x-%02x%02x%02x%02x%02x%02x%02x%02x",
		guid.Data1, guid.Data2, guid.Data3,
		guid.Data4[0], guid.Data4[1], guid.Data4[2],
		guid.Data4[3], guid.Data4[4], guid.Data4[5],
		guid.Data4[6], guid.Data4[7]);
	return std::string(guid_str);
}

inline GUID StringToGuid(std::string guid_str) {
	GUID guid;

	std::vector<std::string> guid_components = split(guid_str, '-');
	if (guid_components.size() == 4) {

		guid.Data1 = std::strtoul(guid_components[0].c_str(), NULL, 16);
		guid.Data2 = (unsigned short)std::strtoul(guid_components[1].c_str(), NULL, 16);
		guid.Data3 = (unsigned short)std::strtoul(guid_components[2].c_str(), NULL, 16);
		//memcpy(guid.Data4, guid_components[3].c_str(), sizeof(unsigned char) * 8);

		const char *suffix = guid_components[3].c_str();
		for (int i = 0; i < 8; i++) {
			char c[2];
			c[0] = suffix[i * 2];
			c[1] = suffix[i * 2 + 1];
			//std::string substr(c);
			guid.Data4[i] = (unsigned char)std::strtoul(c, NULL, 16);
		}

	}

	return guid;

}

// Sample camera capture class
class CLEyeCameraCapture
{
	CHAR _windowName[256];
	GUID _cameraGUID;
	CLEyeCameraColorMode _mode = CLEYE_MONO_PROCESSED;
	CLEyeCameraResolution _resolution = CLEYE_VGA;
	float _fps = 60.f;
	int _currentfps, _fpsCount = 0;
	time_t _lastTime = time(0);
	
	HANDLE _hThread;

	Mat pCapImage;
	PBYTE pCapBuffer = NULL;

	LPCRITICAL_SECTION
		mutex;
public:
	/*
	Callback function to process each captured frame.
	*/
	void(*processFrame)(cv::Mat frame, void *userdata) = NULL;
	void *userdata;


	CLEyeCameraInstance _cam;
	int w, h;
	bool _running;
	int _core; // which processor thread is running on
		
	bool _upsideDown = false;
	bool _flipHorizontally = false;

	float _cameraGain = 0.f;
	float _cameraExposure = 1.f;

public:
	CLEyeCameraCapture(LPSTR windowName, GUID cameraGUID, CLEyeCameraColorMode mode, int processorcore, bool upsidedown, bool flipHorizontally, float cameraGain = 0.0f, float cameraExposure = 1.0f) :
		_cameraGUID(cameraGUID),
		_cam(NULL), _mode(mode),
		_running(false),
		_core(processorcore),
		_upsideDown(upsidedown),
		_flipHorizontally(flipHorizontally),
		_cameraGain(cameraGain),
		_cameraExposure(cameraExposure)
	{
		strcpy(_windowName, windowName);

		_resolution = CLEYE_VGA;
		_fps = 60.f;

		mutex = new CRITICAL_SECTION;

		StartCamera();

	}

	/*
	Apply callback function that will be called on the capturing thread for every frame.
	*/
	void setCallback(void(*callbackFunction)(cv::Mat frame, void *userdata), void *userData) {
		processFrame = callbackFunction;
		userdata = userData;
	}
public:
	void StartCamera() {
		
		if (_cam)
			return;

		// Create camera instance
		_cam = CLEyeCreateCamera(_cameraGUID, _mode, _resolution, _fps);
		if (_cam == NULL)		return;

		// Get camera frame dimensions
		CLEyeCameraGetFrameDimensions(_cam, w, h);

		printf("CLEyeCameraCapture::StartCamera() at resolution %d %d\n", w, h);

		// Depending on color mode chosen, create the appropriate OpenCV image
		if (_mode == CLEYE_COLOR_PROCESSED || _mode == CLEYE_COLOR_RAW) {
			pCapImage = Mat(cvSize(w, h), CV_8UC4);
			pCapImage.zeros(cvSize(w, h), CV_8UC4);
		}

		else {
			pCapImage = Mat(cvSize(w, h), CV_8UC1);
			pCapImage.zeros(cvSize(w, h), CV_8UC1);
		}

		// Set some camera parameters
		//CLEyeSetCameraParameter(_cam, CLEYE_GAIN, int(_cameraGain*79));			// set gain
		//CLEyeSetCameraParameter(_cam, CLEYE_EXPOSURE, int(_cameraExposure*511));	// set exposure
		//CLEyeSetCameraParameter(_cam, CLEYE_AUTO_EXPOSURE, false);						// deactivate auto exposure
		deactivateAutoExposure();
		setGain(_cameraGain);
		setExposure(_cameraExposure);
		
		//CLEyeSetCameraParameter(_cam, CLEYE_GAIN, 59);
		//CLEyeSetCameraParameter(_cam, CLEYE_EXPOSURE, 111);

		// Start capturing
		CLEyeCameraStart(_cam);
		pCapBuffer = pCapImage.data;

		// MUTEX CONDITION
		//InitializeCriticalSection(mutex);

	}
public:
	bool StartCapture()
	{
		
		//cvNamedWindow(_windowName, CV_WINDOW_AUTOSIZE);

		//StartCamera();

		_running = true;

		// MUTEX CONDITION
		InitializeCriticalSection(mutex);

		// Start CLEye image capture thread
		DWORD threadId = 0;
		_hThread = CreateThread(NULL, 0, &CLEyeCameraCapture::CaptureThread, this, 0, &threadId);
		
		//DWORD_PTR c = GetNumCPUs();
		//std::cout << "num processors " << c << std::endl;
		LOGDEBUG("Creating CLEYE camera capture thread (id = %d)\n", threadId);

		// set processor to run the thread on
		DWORD_PTR i = DWORD_PTR(_core);
		DWORD_PTR m_mask = 1 << i; //creates a mask of the processor/core number
		//std::cout << "mask " << m_mask << std::endl;
		
		//DWORD_PTR affinity = SetThreadAffinityMask(_hThread, m_mask);
		BOOL success = SetThreadAffinityMask(_hThread, m_mask);
		if (success == 0)
		{
			LOGWARNING("CLEYE camera capture thread : Setting the Thread Affinity for Main Process could not be done\n");
			//printf("Received CPU = 0x%08x\r\n", affinity);
			//std::cout << "Last error : " << GetLastError() << std::endl;
		}
		else {
			LOGDEBUG("CLEYE camera capture thread : Creating Thread %d (0x%08x) Assigning to CPU 0x%08x\r\n", i, (LONG_PTR)threadId, m_mask); //just so we can see whats going on as it goes on (if your not using unicode, you should use printf here and not wprintf.
		}
		
		//std::cout << "computing on processor " << SetThreadIdealProcessor(_hThread, _core) << std::endl;
		

		if (_hThread == NULL)
		{
			LOGERROR("Could not create CLEYE camera capture thread\n");
			return false;
		}

		


		return true;
	}
	void StopCapture()
	{
		if (!_running)	return;

		_running = false;
		WaitForSingleObject(_hThread, 2000);
		//cvDestroyWindow(_windowName);
		while (_cam) {
			Sleep(100);
			LOGDEBUG("Waiting for camera to stop...\n");
		}
		LOGDEBUG("camera stopped.\n");

	}

	void IncrementCameraParameter(int param)
	{
		if (!_cam)	return;
		printf("CLEyeGetCameraParameter %d\n", CLEyeGetCameraParameter(_cam, (CLEyeCameraParameter)param));
		CLEyeSetCameraParameter(_cam, (CLEyeCameraParameter)param, CLEyeGetCameraParameter(_cam, (CLEyeCameraParameter)param) + 10);
	}

	void DecrementCameraParameter(int param)
	{
		if (!_cam)	return;
		printf("CLEyeGetCameraParameter %d\n", CLEyeGetCameraParameter(_cam, (CLEyeCameraParameter)param));
		CLEyeSetCameraParameter(_cam, (CLEyeCameraParameter)param, CLEyeGetCameraParameter(_cam, (CLEyeCameraParameter)param) - 10);
	}

	int GetCameraParameter(int param)
	{
		printf("CLEyeGetCameraParameter %d\n", CLEyeGetCameraParameter(_cam, (CLEyeCameraParameter)param));
		return CLEyeGetCameraParameter(_cam, (CLEyeCameraParameter)param);
	}

	int GetCurrentFPS()
	{
		
		
		EnterCriticalSection(mutex);
		int currentframes = _currentfps;
		LeaveCriticalSection(mutex);
		return currentframes;
		
		//return _currentfps;
	}

	float getExposure() { return _cameraExposure; }
	bool setExposure(float exposure) {
		
		_cameraExposure = std::min(std::max(0.f, exposure), 1.f); // clamp to [0,1]
		return CLEyeSetCameraParameter(_cam, CLEYE_EXPOSURE, int(_cameraExposure * 511));	// set exposure
	}

	float getGain() { return _cameraGain; }
	bool setGain(float gain) {
	
		_cameraGain = std::min(std::max(0.f, gain), 1.f); // clamp to [0,1]
		return CLEyeSetCameraParameter(_cam, CLEYE_GAIN, int(_cameraGain * 79));			// set gain
	}

	bool deactivateAutoExposure() {
		return CLEyeSetCameraParameter(_cam, CLEYE_AUTO_EXPOSURE, false);						// deactivate auto exposure
	}
	bool activateAutoExposure() {
		return CLEyeSetCameraParameter(_cam, CLEYE_AUTO_EXPOSURE, true);						// activate auto exposure
	}
	
	bool getHorizontalFlip() { return _flipHorizontally; }
	void setHorizontalFlip(bool horflip) {	_flipHorizontally = horflip; }

	bool getUpsideDown() { return _upsideDown; }
	void setUpsideDown(bool upsidedown) { _upsideDown = upsidedown; }

	
	bool setResolution(CLEyeCameraResolution resolution) {
		if (resolution != CLEYE_VGA && resolution != CLEYE_QVGA) // other resolutions not supported
			return false;

		bool restartCamera = this->_running;
		if (restartCamera) {
			StopCapture();	// stop camera thread
		}
		_resolution = resolution;
		if (restartCamera) {
			StartCapture();
		}
		return true;
	}

	bool setFPS(float fps) {
		
		if (_resolution == CLEYE_VGA) {
			if (fps > 75.f || fps < 15.f)
				return false;
		}
		else if (_resolution == CLEYE_QVGA) {
			if (fps > 120.f || fps < 15.f)
				return false;
		}
		
		bool restartCamera = this->_running;
		if (restartCamera) {
			StopCapture();	// stop camera thread
		}
		_fps = fps;
		if (restartCamera) {
			StartCapture();
		}
		return true;
	}


	cv::Size getFrameDimensions() {
		
		int w, h;
		EnterCriticalSection(mutex);
		CLEyeCameraGetFrameDimensions(_cam, w, h);
		LeaveCriticalSection(mutex);
		return cv::Size(w, h);
	}

	cv::Mat getFrame() {

		EnterCriticalSection(mutex);

		Mat frame;
		frame = Mat(cvSize(w, h), CV_8UC1);
		frame.zeros(cvSize(w, h), CV_8UC1);
		pCapImage.copyTo(frame);

		LeaveCriticalSection(mutex);

		return frame;
	}

	void Run()
	{

		_currentfps = 0;
		_fpsCount = 0;
		_lastTime = time(0);

		// image capturing loop
		while (_running && _cam)
		{
			CLEyeCameraGetFrame(_cam, pCapBuffer);

			EnterCriticalSection(mutex);

			// current FPS computation
			_fpsCount++;

			if (time(0) > _lastTime) {
				_currentfps = _fpsCount;
				_fpsCount = 0;
				_lastTime = time(0);

				//printf("fps = %d\n", _currentfps);
			}
			LeaveCriticalSection(mutex);

			// if callback function is set, return image to the function
			//EnterCriticalSection(mutex);
			if (processFrame) {

				
				cv::Mat frame;

				//if (_mode == CLEYE_COLOR_PROCESSED || _mode == CLEYE_COLOR_RAW) {
				//	frame = Mat(cvSize(w, h), CV_8UC4);
				//	frame.zeros(cvSize(w, h), CV_8UC4);
				//}

				//else {
					frame = Mat(cvSize(w, h), CV_8UC1);
					frame.zeros(cvSize(w, h), CV_8UC1);
				//}

				pCapImage.copyTo(frame);


				// transpoe frame from landscape to portrait
				transpose(frame, frame);

				if (_upsideDown) // only works with following flip function !!
					flip(frame, frame, 0);
				if (!_flipHorizontally)
					flip(frame, frame, 1);

				

				processFrame(frame, userdata);

			}
			//LeaveCriticalSection(mutex);
	
			//Sleep(10);

		}
		// Stop camera capture
		CLEyeCameraStop(_cam);
		// Destroy camera object
		CLEyeDestroyCamera(_cam);

		pCapImage.release();

		_cam = NULL;
	}

	std::string getDeviceID() {

		//char id[1024];
		//sprintf(id, "%08lX-%04hX-%04hX-%02hhX%02hhX-%02hhX%02hhX%02hhX%02hhX%02hhX%02hhX",
		//	_cameraGUID.Data1, _cameraGUID.Data2, _cameraGUID.Data3,
		//	_cameraGUID.Data4[0], _cameraGUID.Data4[1], _cameraGUID.Data4[2], _cameraGUID.Data4[3],
		//	_cameraGUID.Data4[4], _cameraGUID.Data4[5], _cameraGUID.Data4[6], _cameraGUID.Data4[7]);
		//return std::string(id);

		std::string guidstr = GUIDToString(_cameraGUID);
		return guidstr;

	}

	static DWORD WINAPI CaptureThread(LPVOID instance)
	{
		// seed the rng with current tick count and thread id
		srand(GetTickCount() + GetCurrentThreadId());
		// forward thread to Capture function
		CLEyeCameraCapture *pThis = (CLEyeCameraCapture *)instance;
		pThis->Run();
		return 0;
	}
};
