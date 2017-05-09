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

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "CLEyeCameraCapture.h"

#include <vector>
#include <string>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <Windows.h>

using namespace cv;
using namespace std;

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
	);

// ***************************************************
// CAMERA DATA

static CLEyeCameraCapture *VidCapLeft;
static CLEyeCameraCapture *VidCapRight;

static bool newframe_left = false;
static bool newframe_right = false;
static cv::Mat current_frame_left;
static cv::Mat current_frame_right;
static void *userdata_left;
static void *userdata_right;
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

static void cameracallback_left(cv::Mat frame, void *userdata) {

	//transpose(frame, frame);
	flip(frame, frame, 0);
	flip(frame, frame, 1);

	frame.copyTo(current_frame_left);
	newframe_left = true;

}

static void cameracallback_right(cv::Mat frame, void *userdata) {

	//transpose(frame, frame);
	//flip(frame, frame, 1);

	frame.copyTo(current_frame_right);
	newframe_right = true;
}

bool startCameras() {

	int numCams = CLEyeGetCameraCount();

	if (numCams < 2) {
		cout << "No or not enough cameras for dual cam mode connected !" << endl;
		return false;
	}

	char windowName[64];
	// Query unique camera uuid
	GUID guid_left = CLEyeGetCameraUUID(0);
	GUID guid_right = CLEyeGetCameraUUID(1);
	
	// Create camera capture object
	VidCapLeft = new CLEyeCameraCapture(windowName, guid_left, CLEYE_COLOR_PROCESSED, 0, false, false);
	VidCapLeft->setCallback(cameracallback_left, userdata_left);

	VidCapRight = new CLEyeCameraCapture(windowName, guid_right, CLEYE_COLOR_PROCESSED, 1, false, false);
	VidCapRight->setCallback(cameracallback_right, userdata_right);

	return (VidCapLeft->StartCapture() && VidCapRight->StartCapture());

}

void checkCameraFrames(vector<Point2f> &corners_left, vector<Point2f> &corners_right, cv::Mat &left, cv::Mat &right, cv::Mat &combined) {

	if (!newframe_left || !newframe_right)
		return;

	newframe_left = false;
	newframe_right = false;

	//std::cout << "cam size " << w << "x" << h << std::endl;

	left_checker = combined(Rect(0, 0, w, h));
	right_checker = combined(Rect(w, 0, w, h));

	{
		current_frame_left.copyTo(left_checker);
		current_frame_left.copyTo(left);

		corners_left.clear();

		bool found = findChessboardCorners(left_checker, boardSize, corners_left,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			drawChessboardCorners(left_checker, boardSize, corners_left, found);
		}

		//imshow("corners left", left_checker);
	}

	{
		current_frame_right.copyTo(right_checker);
		current_frame_right.copyTo(right);

		corners_right.clear();

		bool found = findChessboardCorners(right_checker, boardSize, corners_right,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		if (found) {
			drawChessboardCorners(right_checker, boardSize, corners_right, found);
		}

		//imshow("corners right", right_checker);
	}

	imshow("stereo camera feed", combined);
	waitKey(1);

	/*
	left_checker.copyTo(canvasPart_left);
	right_checker.copyTo(canvasPart_right);

	for (int j = 0; j < canvas.rows; j += 16)
		line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

	imshow("rectified", canvas);

	//cvWaitKey(1);
	*/


	return;
}

// ***************************************************

static void StereoCalibOnline() {

	// *******************************************************************************
	// INITIALIZATION

	// pick stereo pairs
	vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;

	imgSamplesLeft.clear();
	imgSamplesRight.clear();

	int nimages = 15;

	

	imageSize.height = 640;
	imageSize.width = 480;

	w = imageSize.width;
	h = imageSize.height;

	cv::Mat left, right, combined;
	combined = Mat(h, 2 * w, CV_8UC(4));

	std::cout << "Press 'C' key to capture stereo image pair of checkerboard" << std::endl;

	// *******************************************************************************
	// SAMPLE RECORDINGS

	int numsamples = 0;
	do {

		vector<Point2f> corners_left, corners_right;
		

		checkCameraFrames(corners_left, corners_right, left, right, combined);

		if (corners_left.size() > 0 && corners_right.size() > 0) {

			if (GetAsyncKeyState('C') & 0x8000) {
				numsamples++;
				imagePoints[0].push_back(corners_left);
				imagePoints[1].push_back(corners_right);

				// save current frames
				imgSamplesLeft.push_back(left);
				imgSamplesRight.push_back(right);

				std::cout << "sample " << numsamples << " recorded. " << nimages - numsamples << " to go..." << std::endl;
				Sleep(100);
			}
		}

	} while (numsamples < nimages);

	// *******************************************************************************
	// STEREO CALIBRATION

	// transform 2d correspondences to 3d world points
	imagePoints[0].resize(nimages);
	imagePoints[1].resize(nimages);
	objectPoints.resize(nimages);

	int i, j, k;

	for (i = 0; i < nimages; i++)
	{
		for (j = 0; j < boardSize.height; j++)
			for (k = 0; k < boardSize.width; k++)
				objectPoints[i].push_back(Point3f(j*squareSize, k*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";

	// compute intrinsics

	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);

	//imageSize.height = 640;
	//imageSize.width = 480;

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_ZERO_TANGENT_DIST +
		CV_CALIB_SAME_FOCAL_LENGTH +
		CV_CALIB_RATIONAL_MODEL +
		CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5 //+CV_CALIB_FIX_INTRINSIC
		,TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5)
		);
	cout << "done with RMS error=" << rms << endl;


	// save intrinsic parameters
	FileStorage fs("intrinsics.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";


	// compute and save extrinsics

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	fs.open("extrinsics.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	// *******************************************************************************

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], k + 1, F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
					imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average reprojection err = " << err / npoints << endl;

	// *******************************************************************************
	// SETUP IMAGE RECTIFICATION

	bool useCalibrated = false;


	// IF BY CALIBRATED (BOUGUET'S METHOD)
	if (useCalibrated)
	{
		// we already computed everything
	}
	// OR ELSE HARTLEY'S METHOD
	else
		// use intrinsic parameters of each camera, but
		// compute the rectification transformation directly
		// from the fundamental matrix
	{
		vector<Point2f> allimgpt[2];
		for (k = 0; k < 2; k++)
		{
			for (i = 0; i < nimages; i++)
				std::copy(imagePoints[k][i].begin(), imagePoints[k][i].end(), back_inserter(allimgpt[k]));
		}
		F = findFundamentalMat(Mat(allimgpt[0]), Mat(allimgpt[1]), FM_8POINT, 0, 0);
		Mat H1, H2;
		stereoRectifyUncalibrated(Mat(allimgpt[0]), Mat(allimgpt[1]), F, imageSize, H1, H2, 3);

		R1 = cameraMatrix[0].inv()*H1*cameraMatrix[0];
		R2 = cameraMatrix[1].inv()*H2*cameraMatrix[1];
		P1 = cameraMatrix[0];
		P2 = cameraMatrix[1];
	}

	//Precompute maps for cv::remap()
	initUndistortRectifyMap(cameraMatrix[0], distCoeffs[0], R1, P1, imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
	initUndistortRectifyMap(cameraMatrix[1], distCoeffs[1], R2, P2, imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

	fs.open("rectification.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "rmap00" << rmap[0][0]
			<< "rmap01" << rmap[0][1]
			<< "rmap10" << rmap[1][0]
			<< "rmap11" << rmap[1][1]
			<< "imageSize" << imageSize;
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	// **********************************************************************************************************

	w = imageSize.width;
	h = imageSize.height;

	canvas = Mat(h, 2 * w, CV_8UC(4));

	canvasPart_left = canvas(Rect(0, 0, w, h));
	canvasPart_right = canvas(Rect(w, 0, w, h));

#if 0

	for (i = 0; i < nimages; i++)
	{

		img_l = imgSamplesLeft[i]; // copy image from sample vector
		img_r = imgSamplesRight[i];// copy image from sample vector

		remap(img_l, rimg_l, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
		remap(img_r, rimg_r, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);

		rimg_l.copyTo(canvasPart_left);
		rimg_r.copyTo(canvasPart_right);

		// draw some lines to evaluate rectification quality
		for (j = 0; j < canvas.rows; j += 16)
			line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

		imshow("rectified", canvas);
		char c = (char)waitKey(0);

	}
#endif

	destroyAllWindows();
}

/*
static bool readStringList( const string& filename, vector<string>& l )
{
l.resize(0);
FileStorage fs(filename, FileStorage::READ);
if( !fs.isOpened() )
return false;
FileNode n = fs.getFirstTopLevelNode();
if( n.type() != FileNode::SEQ )
return false;
FileNodeIterator it = n.begin(), it_end = n.end();
for( ; it != it_end; ++it )
l.push_back((string)*it);
return true;
}
*/

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

	canvas = Mat(h, 2 * w, CV_8UC(4));

	canvasPart_left = canvas(Rect(0, 0, w, h));
	canvasPart_right = canvas(Rect(w, 0, w, h));

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

	for (int j = 0; j < canvas.rows; j += 16)
		line(canvas, Point(0, j), Point(canvas.cols, j), Scalar(0, 255, 0), 1, 8);

	imshow("rectified", canvas);
	waitKey(1);

	return true;

}

int main(int argc, char** argv)
{

	// start cameras
	if (!startCameras()) {
		cout << "Exiting.";
		return 0;
	}


	// read rectification information
	if (!readCalibration()) {
		cout << "no calibration existing or not readable. perform new calibration ? ('Y' / 'N' )";
		char c; cin >> c;
		if (c == 'y' || c == 'Y')
			StereoCalibOnline();// perform stereo calibration
		else {
			cout << "Exiting.";
			return 0;
		}
	}

	initRectification();

	while (true) {

		rectifyCameraImages();

		if ((GetAsyncKeyState('C') & 0x8000 || GetAsyncKeyState('c') & 0x8000)
			&& GetAsyncKeyState(VK_CONTROL) & 0x8000
			&& GetAsyncKeyState(VK_SHIFT) & 0x8000
			) {
			cout << "run calibration" << endl;
			destroyAllWindows();

			// perform stereo calibration
			StereoCalibOnline();
		}

		Sleep(5);
	}

	// run validation by image rectification
	return 0;

#if 0

	Size boardSize;
	string imagelistfn;
	bool showRectified = true;

	for (int i = 1; i < argc; i++)
	{
		if (string(argv[i]) == "-w")
		{
			if (sscanf(argv[++i], "%d", &boardSize.width) != 1 || boardSize.width <= 0)
			{
				cout << "invalid board width" << endl;
				return print_help();
			}
		}
		else if (string(argv[i]) == "-h")
		{
			if (sscanf(argv[++i], "%d", &boardSize.height) != 1 || boardSize.height <= 0)
			{
				cout << "invalid board height" << endl;
				return print_help();
			}
		}
		else if (string(argv[i]) == "-nr")
			showRectified = false;
		else if (string(argv[i]) == "--help")
			return print_help();
		else if (argv[i][0] == '-')
		{
			cout << "invalid option " << argv[i] << endl;
			return 0;
		}
		else
			imagelistfn = argv[i];
	}

	if (imagelistfn == "")
	{
		imagelistfn = "stereo_calib.xml";
		boardSize = Size(9, 6);
	}
	else if (boardSize.width <= 0 || boardSize.height <= 0)
	{
		cout << "if you specified XML file with chessboards, you should also specify the board width and height (-w and -h options)" << endl;
		return 0;
	}

	vector<string> imagelist;
	bool ok = readStringList(imagelistfn, imagelist);
	if (!ok || imagelist.empty())
	{
		cout << "can not open " << imagelistfn << " or the string list is empty" << endl;
		return print_help();
	}

	StereoCalib(imagelist, boardSize, true, showRectified);
	return 0;
#endif

}
