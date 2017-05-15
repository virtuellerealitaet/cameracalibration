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

void checkCameraFrames(vector<Point2f> &corners_left, vector<Point2f> &corners_right, cv::Mat &left, cv::Mat &right, cv::Mat &combined) {


	receiveCameraFrames();
	
	// adjust image regions for left and right camera frame in combined visualization frame
	left_checker = combined(Rect(0, 0, w, h));
	right_checker = combined(Rect(w, 0, w, h));

	{
		// copy camera frame
		current_frame_left.copyTo(left_checker);
		current_frame_left.copyTo(left);

		// clear corner points of calibration pattern for left camera
		corners_left.clear();

		bool found = findChessboardCorners(left_checker, boardSize, corners_left,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		// draw chessboard corners if find function was successful
		if (found) { drawChessboardCorners(left_checker, boardSize, corners_left, found); }
	}

	{
		// copy camera frame
		current_frame_right.copyTo(right_checker);
		current_frame_right.copyTo(right);

		// clear corner points of calibration pattern for right camera
		corners_right.clear();

		bool found = findChessboardCorners(right_checker, boardSize, corners_right,
			CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

		// draw chessboard corners if find function was successful
		if (found) { drawChessboardCorners(right_checker, boardSize, corners_right, found);	}
				
	}

	// show combined visualization image
	imshow("stereo camera feed", combined);
	waitKey(1);
	
	return;
}

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
	combined = Mat(h, 2 * w, CV_8UC(3));

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


	// save calibration points parameters
	FileStorage fs("calibrationpoints.yml", CV_STORAGE_WRITE);
	if (fs.isOpened())
	{
		fs << "numImages" << nimages;
		fs << "objectPoints" << objectPoints;
		fs << "imagePoints1" << imagePoints[0];
		fs << "imagePoints2" << imagePoints[1];

		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	cout << "Running stereo calibration ...\n";

	// compute intrinsics

	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);

	double rms = stereoCalibrate(
		objectPoints,					// Vector of vectors of the calibration pattern points
		imagePoints[0],					// Vector of vectors of the projections of the calibration pattern points, observed by the first camera
		imagePoints[1],					// Vector of vectors of the projections of the calibration pattern points, observed by the second camera
		cameraMatrix[0],				// Input/output first camera matrix
		distCoeffs[0],					// Input/output vector of distortion coefficients 
		cameraMatrix[1],				// input / output second camera matrix.The parameter is similar to cameraMatrix1
		distCoeffs[1],					// 	Input/output lens distortion coefficients for the second camera. The parameter is similar to distCoeffs1
		imageSize,						// Size of the image used only to initialize intrinsic camera matrix.
		R,								// Output rotation matrix between the 1st and the 2nd camera coordinate systems.
		T,								// Output translation vector between the coordinate systems of the cameras.
		E,								// Output essential matrix.
		F,								// Output fundamental matrix.
		CV_CALIB_FIX_ASPECT_RATIO +
		CV_CALIB_ZERO_TANGENT_DIST +	// Set tangential distortion coefficients for each camera to zeros and fix there.
		CV_CALIB_SAME_FOCAL_LENGTH +	// Enforce f(0)x=f(1)x and f(0)y=f(1)y .
		CV_CALIB_RATIONAL_MODEL +		// Enable coefficients k4, k5, and k6.
		CV_CALIB_FIX_K3 + CV_CALIB_FIX_K4 + CV_CALIB_FIX_K5	// Do not change the corresponding radial distortion coefficient during the optimization.
		//+CV_CALIB_FIX_INTRINSIC
		,TermCriteria(CV_TERMCRIT_ITER + CV_TERMCRIT_EPS, 100, 1e-5)	// Termination criteria for the iterative optimization algorithm.
		);
	cout << "done with RMS error=" << rms << endl;


	// save intrinsic parameters
	{
		fs.open("intrinsics.yml", CV_STORAGE_WRITE);
		if (fs.isOpened())
		{
			fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
				"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
			fs.release();
		}
		else
			cout << "Error: can not save the intrinsic parameters\n";
	}

	// compute and save extrinsics

	stereoRectify(
		cameraMatrix[0],		// first camera matrix
		distCoeffs[0],			// first camera lens distortion parameters
		cameraMatrix[1],		// second camera matrix
		distCoeffs[1],			// second camera lens distortion parameters
		imageSize,				// image size used for stereo calibration
		R,						// rotation matrix between cameras
		T,						// translation vector between cameras
		R1, R2,					// output of rectification matrices for first and second camera
		P1, P2,					// output of projection matrices for first and second camera
		Q,						// 4x4 disparity-to-depth mapping matrix (used for reprojectImageTo3D () )
		CALIB_ZERO_DISPARITY,	// principal points for both views have the same pixel coordinate in rectified views, without flag images can still be shifted !
		1,						// alpha for scaling (-1 default, 0 -> zoom in so that only valid pixels are visible, 1 -> all pixels remain visible
		imageSize,				// new image resolution after rectification, same should be given to initUndistortRectifyMap()
		&validRoi[0],			// for visualization of valid pixels in view of first camera
		&validRoi[1]			// for visualization of valid pixels in view of second camera
		);
	
	// save extrinsics

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

	destroyAllWindows();
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

	while (true)
	{

		receiveCameraFrames();

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

	// deinitialize both cameras	
	VidCapLeft->deinitialize();
	VidCapRight->deinitialize();

	// run validation by image rectification
	return 0;

}
