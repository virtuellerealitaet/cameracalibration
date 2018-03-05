

#include <iostream>
#include <vector>

#include <Windows.h>

// OpenCV
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
);

//static void loadCameraParams(const string& filename,
//	Size imageSize, Size boardSize,
//	float squareSize, float aspectRatio, int flags,
//	const Mat& cameraMatrix, const Mat& distCoeffs,
//	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
//	const vector<float>& reprojErrs,
//	const vector<vector<Point2f> >& imagePoints,
//	double totalAvgErr)

struct CameraCalibration
{
	string time;
	int calibrationImageWidth;
	int calibrationImageHeight;
	int numberOfCrossPointsInWidth;
	int numberOfCrossPointsInHeight;
	double squareSize;
	int numberOfCalibratedImages;
	Mat cameraMatrix;
	Mat distortionCoefficient;

	cv::Size imageSize;

};

static bool loadCameraParams(const string& filename, CameraCalibration& c)
{
	FileStorage fs(filename, FileStorage::READ);

	// Read data
	FileNode fn_time = fs.root();

	c.calibrationImageWidth = fs["image_width"];
	c.calibrationImageHeight = fs["image_height"];
	c.numberOfCrossPointsInWidth = fs["board_width"];
	c.numberOfCrossPointsInHeight = fs["board_height"];
	c.squareSize = fs["square_size"];
	c.numberOfCalibratedImages = fs["nframes"];

	fs["camera_matrix"] >> c.cameraMatrix;
	fs["distortion_coefficients"] >> c.distortionCoefficient;

	c.imageSize = cv::Size(c.calibrationImageWidth, c.calibrationImageHeight);

	return true;
}



int main(int argc, char *argv[])
{

	std::string calibrationFile = std::string("calibration");
	for (int i = 0; i < argc; i++)
	{
		std::cout << i << " " << std::string(argv[i]).c_str() << std::endl;
	}
	if (argc > 1)
	{
		calibrationFile = std::string(argv[1]);
	}
	std::cout << calibrationFile << std::endl;

	// load camera calibration
	CameraCalibration c;
	if (!loadCameraParams(calibrationFile, c))
	{
		printf("ERROR: Could not read calibration file '%s'\n", calibrationFile.c_str());
		return -1;
	}

	// setup undistortion
	//float alpha = 0.0; // returns undistorted image with minimum unwanted pixels
	float alpha = 1.0; // all pixels are retained with some extra black images
	cv::Mat roi = getOptimalNewCameraMatrix(c.cameraMatrix, c.distortionCoefficient, c.imageSize, alpha); // compute region of interest for undistortion using specified alpha
	Mat mapx, mapy;
	initUndistortRectifyMap(c.cameraMatrix, c.distortionCoefficient, Mat(), roi, c.imageSize, CV_16SC2, mapx, mapy);

	// open OpenCV camera
	int cameraId = 0;
	cv::VideoCapture camera(0);
	if (!camera.isOpened())
	{
		printf("ERROR: Could not open OpenCV camera\n");
		return -1;
	}

	camera.set(CV_CAP_PROP_FRAME_WIDTH, c.calibrationImageWidth);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, c.calibrationImageHeight);
	printf("received camera resolution : %d x %d\n", (int)camera.get(CV_CAP_PROP_FRAME_WIDTH), (int)camera.get(CV_CAP_PROP_FRAME_HEIGHT));
	
	if (c.calibrationImageWidth != (int)camera.get(CV_CAP_PROP_FRAME_WIDTH) || c.calibrationImageHeight != (int)camera.get(CV_CAP_PROP_FRAME_HEIGHT))
	{
		printf("ERROR: Camera resolution is different from camera calibrated resolution (%d x %d vs %d x %d)\n", c.calibrationImageWidth, c.calibrationImageHeight, (int)camera.get(CV_CAP_PROP_FRAME_WIDTH), (int)camera.get(CV_CAP_PROP_FRAME_HEIGHT));
		return -1;
	}




	// open camera window
    std::string windowName("camera");
	cv::namedWindow(windowName, 1);
	std::cout << "Beginning camera loop (press key to stop process)" << std::endl;
	cv::Mat view = cv::Mat(cv::Size(c.calibrationImageWidth, c.calibrationImageHeight), CV_8UC3);

	int frameNumber = 0;

    while (true)
    {
		while (true)
		{
			camera >> view;

			if (view.size().empty())
			{
				printf("no new camera frame yet\n");
				Sleep(100);
			}
			else
			{
				break;
			}
		}

		cv::Mat undistortedImage = view.clone();
		cv::remap(view, undistortedImage, mapx, mapy, INTER_LINEAR);

		// detect checkerboard


		// show image
		cv::imshow(windowName, undistortedImage);

        // quite program on keyboard input
		char key = cvWaitKey(1);
		if ((key & 255) == 27)
			break;
    }

	std::cout << "\nCamera loop stopped.\n\n";

	std::cout << "Disconnecting camera ...\n";

	// deinitialize camera
	camera.release();

    cv::destroyAllWindows();

	std::cout << "done.\n\nRegular program exit.\n" << std::endl;

    return 0;
}

