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

#include "CameraPS3Eye.h"

using namespace cv;
using namespace std;

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
	);

// ***************************************************
// CAMERA DATA

static CameraPS3Eye *VidCapLeft;
static CameraPS3Eye *VidCapRight;

static bool newframe_left = false;
static bool newframe_right = false;
static cv::Mat current_frame_left;
static cv::Mat current_frame_right;

static bool switchcameras = false;

// ***************************************************

// frame copies augmented by checkerboard corners
static Mat left_checker;
static Mat right_checker;

std::vector<cv::Mat> imgSamplesLeft;
std::vector<cv::Mat> imgSamplesRight;

// ***************************************************
// CALIBRATION DATA

static Size boardSize(9, 6);	// Set this to your actual checker board pattern (number of squares)
static float squareSize = 1.f;  // Set this to your actual square size

static Mat cameraMatrix[2], distCoeffs[2];
static Mat R, T, E, F;
static Mat R1, R2, P1, P2, Q;
static Mat rmap[2][2];
static Size imageSize;

// ***************************************************
// TEMPORARY DATA

static Rect validRoi[2];
static Mat canvas;
static int w, h;

static Mat img_l, rimg_l, cimg_l;
static Mat img_r, rimg_r, cimg_r;

static Mat canvasPart_left;
static Mat canvasPart_right;

Point mouseCoord;

bool startCameras()
{

	// list out the devices
	using namespace ps3eye;
	std::vector<PS3EYECam::PS3EYERef> devices(PS3EYECam::getDevices());
	LOGCON("Found %d cameras.\n", (int)devices.size());

	int numCams = (int)devices.size();

	if (numCams < 2) {
		cout << "No or not enough cameras for dual cam mode connected !" << endl;
		return false;
	}

	// create and initialize two sony ps3 eye cameras
	VidCapLeft = new CameraPS3Eye();
	VidCapRight = new CameraPS3Eye();
	bool success = (VidCapLeft->initialize(640,480,3,60,0) && VidCapRight->initialize(640, 480, 3, 60, 1));

	// adjust camera settings
	if (success)
	{
		VidCapLeft->_autogain = true;
		VidCapLeft->_autowhitebalance = true;
		VidCapLeft->_flipVertically = true;
		VidCapLeft->updateCameraSettings();

		VidCapRight->_autogain = true;
		VidCapRight->_autowhitebalance = true;
		VidCapRight->_flipHorizontally = true;
		VidCapRight->updateCameraSettings();
	}

	return success;
}

// update frames from both cameras
void receiveCameraFrames()
{

	current_frame_left = VidCapLeft->receiveFrame();
	current_frame_right = VidCapRight->receiveFrame();

	transpose(current_frame_left, current_frame_left);
	transpose(current_frame_right, current_frame_right);

	newframe_left = true;
	newframe_right = true;

}

static bool readCalibration() {

	// load intrinsics
	FileStorage fs("intrinsics.yml", CV_STORAGE_READ);
	if (fs.isOpened())
	{
		fs["M1"] >> cameraMatrix[0];
		fs["D1"] >> distCoeffs[0];
		fs["M2"] >> cameraMatrix[1];
		fs["D2"] >> distCoeffs[1];

		fs.release();
	}
	else {
		cout << "Error: can not load the extrinsics parameters\n";
		return false;
	}

	// load extrinsics
	fs.open("extrinsics.yml", CV_STORAGE_READ);
	if (fs.isOpened())
	{
		fs["R"] >> R;
		fs["T"] >> T;
		fs["R1"] >> R1;
		fs["R2"] >> R2;
		fs["P1"] >> P1;
		fs["P2"] >> P2;
		fs["Q"] >> Q;
		fs.release();
	}
	else {
		cout << "Error: can not load the intrinsic parameters\n";
		return false;
	}

	// load rectification map
	fs.open("rectification.yml", CV_STORAGE_READ);
	if (fs.isOpened())
	{
		fs["rmap00"] >> rmap[0][0];
		fs["rmap01"] >> rmap[0][1];
		fs["rmap10"] >> rmap[1][0];
		fs["rmap11"] >> rmap[1][1];
		fs["imageSize"] >> imageSize;
		fs.release();
	}
	else {
		cout << "Error: can not load the rectification maps\n";
		return false;
	}

	return true;
}

static void initRectification() {

	w = imageSize.width;
	h = imageSize.height;

	canvas = Mat(h, 2 * w, CV_8UC(3));

	canvasPart_left = canvas(Rect(0, 0, w, h));
	canvasPart_right = canvas(Rect(w, 0, w, h));


	// recompute rectification

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

}

static bool rectifyCameraImages() {

	if (!newframe_left || !newframe_right)
		return false;

	newframe_left = false;
	newframe_right = false;

	remap(current_frame_left, rimg_l, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
	remap(current_frame_right, rimg_r, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);

	rimg_l.copyTo(canvasPart_left);
	rimg_r.copyTo(canvasPart_right);

	//for (int j = 0; j < canvas.rows; j += 16)
	//	line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

	line(canvas, Point(0, mouseCoord.y), Point(canvas.cols, mouseCoord.y), Scalar(0, 255, 0), 1, 8);




	return true;

}

void onMouse(int event, int x, int y, int, void*)
{
	mouseCoord = Point(x, y);
}

int main(int argc, char** argv)
{

	// start cameras
	if (!startCameras()) {
		cout << "Exiting.";
		return 0;
	}
	
	// read rectification information
	if (!readCalibration())
	{
		cout << "no calibration existing or not readable. exit.";
		cout << "Exiting.";
		return 0;
	}

	initRectification();

	// register mouse event callback for new window
	namedWindow("rectified");
	setMouseCallback("rectified", onMouse, 0);


	while (true)
	{

		receiveCameraFrames();

		rectifyCameraImages();

		imshow("rectified", canvas);
		waitKey(1);

		Sleep(5);
	}

	destroyAllWindows();

	// deinitialize both cameras	
	VidCapLeft->deinitialize();
	VidCapRight->deinitialize();

	// run validation by image rectification
	return 0;

}
