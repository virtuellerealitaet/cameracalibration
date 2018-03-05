

#include <iostream>
#include <vector>

#include <Windows.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

using namespace cv;
using namespace std;

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
);


enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

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

	cv::Size imageSize, boardSize;

	Pattern pattern = CHESSBOARD;
	vector<vector<Point3f> > objectPoints;

};

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
	corners.resize(0);

	switch (patternType)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float(j*squareSize),
					float(i*squareSize), 0));
		break;

	case ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float((2 * j + i % 2)*squareSize),
					float(i*squareSize), 0));
		break;

	default:
		CV_Error(CV_StsBadArg, "Unknown pattern type\n");
	}
}

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
	c.boardSize = cv::Size(c.numberOfCrossPointsInWidth, c.numberOfCrossPointsInHeight);

	c.objectPoints = vector<vector<Point3f>>(1);
	
	calcChessboardCorners(c.boardSize, c.squareSize, c.objectPoints[0], c.pattern);

	return true;
}

void drawAxis(cv::Mat _image, cv::Mat _cameraMatrix, cv::Mat _distCoeffs,
	cv::Mat _rvec, cv::Mat _tvec, float length) {

	CV_Assert(_image.total() != 0 && (_image.channels() == 1 || _image.channels() == 3));
	CV_Assert(length > 0);

	// project axis points
	vector< Point3f > axisPoints;
	axisPoints.push_back(Point3f(0, 0, 0));
	axisPoints.push_back(Point3f(length, 0, 0));
	axisPoints.push_back(Point3f(0, length, 0));
	axisPoints.push_back(Point3f(0, 0, length));
	vector< Point2f > imagePoints;
	projectPoints(axisPoints, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);

	// draw axis lines
	line(_image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 3);
	line(_image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 3);
	line(_image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 3);
		
	cv::String msg = format("world origin X %.2f, Y %.2f, Z %.2f", _tvec.at<double>(0), _tvec.at<double>(1), _tvec.at<double>(2));
	cv::Point textOrigin = cv::Point(imagePoints[0]) + cv::Point(0, 0);
	
	int baseLine = 0;
	Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);

	// black background behind text
	int x, y, w, h;
	x = std::max(0,textOrigin.x);
	y = std::max(0, -10 + textOrigin.y);
	w = std::max(0, textSize.width);
	h = std::max(0, 20);

	w = std::min(_image.cols-x, w);
	h = std::min(_image.rows-y, h);
		
	cv::Rect roi_rect(x, y, w, h);

	{
		cv::Mat roi(_image, roi_rect);
		roi.setTo(Scalar(0));
		putText(_image, msg, textOrigin, 1, 1, Scalar(0, 255, 0));
	}

	
}

void computeExtrinsics(vector<Point2f> &pointbuf, CameraCalibration &calib, cv::Size &boardSize, float squareSize, cv::Mat &rotationVector, cv::Mat &translationVector)
{

	vector<Point3f> objectPoints;
	calcChessboardCorners(boardSize, squareSize, objectPoints);

	vector<Point2f> objectPointsPlanar;
	for (size_t i = 0; i < objectPoints.size(); i++)
	{
		objectPointsPlanar.push_back(Point2f(objectPoints[i].x, objectPoints[i].y));
	}

	vector<Point2f> imagePoints;

	// not required if we are working on an undistorted frame
	undistortPoints(pointbuf, imagePoints, calib.cameraMatrix, calib.distortionCoefficient);

	Mat H = findHomography(objectPointsPlanar, imagePoints);
	//cout << "H:\n" << H << endl;
	// Normalization to ensure that ||c1|| = 1
	double norm = sqrt(H.at<double>(0, 0)*H.at<double>(0, 0) +
		H.at<double>(1, 0)*H.at<double>(1, 0) +
		H.at<double>(2, 0)*H.at<double>(2, 0));
	H /= norm;
	Mat c1 = H.col(0);
	Mat c2 = H.col(1);
	Mat c3 = c1.cross(c2);
	Mat tvec = H.col(2);
	Mat R(3, 3, CV_64F);
	for (int i = 0; i < 3; i++)
	{
		R.at<double>(i, 0) = c1.at<double>(i, 0);
		R.at<double>(i, 1) = c2.at<double>(i, 0);
		R.at<double>(i, 2) = c3.at<double>(i, 0);
	}
	//cout << "R (before polar decomposition):\n" << R << "\ndet(R): " << determinant(R) << endl;
	Mat W, U, Vt;
	SVDecomp(R, W, U, Vt);
	R = U*Vt;
	//cout << "R (after polar decomposition):\n" << R << "\ndet(R): " << determinant(R) << endl;
	
	Rodrigues(R, rotationVector);
	translationVector = tvec;

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

	c.squareSize = 6.f;


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

	Size cameraImageSize = Size((int)camera.get(CV_CAP_PROP_FRAME_WIDTH), (int)camera.get(CV_CAP_PROP_FRAME_HEIGHT));
	
	//Size undistortedSize = cameraImageSize;
	Size undistortedSize = c.imageSize;

	// setup undistortion
	float alpha = 0.0; // returns undistorted image with minimum unwanted pixels
	//float alpha = 1.0; // all pixels are retained with some extra black images
	cv::Mat roi = getOptimalNewCameraMatrix(c.cameraMatrix, c.distortionCoefficient, undistortedSize, alpha); // compute region of interest for undistortion using specified alpha
	Mat mapx, mapy;
	initUndistortRectifyMap(c.cameraMatrix, c.distortionCoefficient, Mat(), roi, undistortedSize, CV_16SC2, mapx, mapy);



	// open camera window
    std::string windowName("camera");
	cv::namedWindow(windowName, 1);
	std::cout << "Beginning camera loop (press key to stop process)" << std::endl;
	cv::Mat view = cv::Mat(cv::Size(c.calibrationImageWidth, c.calibrationImageHeight), CV_8UC3);

	int frameNumber = 0;

	int trackingMode = 1; // 0 = single tracking mode, 1 = dual tracking mode

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

		if (trackingMode == 0)
		{
			// detect checkerboard
			vector<Point2f> pointbuf;
			bool found;
			switch (c.pattern)
			{
				case CHESSBOARD:
					found = findChessboardCorners(undistortedImage, c.boardSize, pointbuf,
						CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
					break;
				case CIRCLES_GRID:
					found = findCirclesGrid(view, c.boardSize, pointbuf);
					break;
				case ASYMMETRIC_CIRCLES_GRID:
					found = findCirclesGrid(view, c.boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
					break;
				default:
					return fprintf(stderr, "Unknown pattern type\n"), -1;
			}

			// improve the found corners' coordinate accuracy
			bool refinement = true;
			if (refinement)
			{
				Mat viewGray;
				cvtColor(undistortedImage, viewGray, CV_BGR2GRAY);
				if (c.pattern == CHESSBOARD && found) cornerSubPix(viewGray, pointbuf, Size(11, 11),
					Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
			}
		
			if (found)
			{
				drawChessboardCorners(undistortedImage, c.boardSize, Mat(pointbuf), found);

				cv::Mat rotationVector, translationVector;
				computeExtrinsics(pointbuf, c, c.boardSize, c.squareSize, rotationVector, translationVector);

				drawAxis(undistortedImage, c.cameraMatrix, c.distortionCoefficient, rotationVector, translationVector, 2 * c.squareSize);
			}
		}
		else if (trackingMode == 1)
		{

			if (undistortedImage.empty())
				continue;

			cv::Mat rotationVector[2], translationVector[2];

			cv::Mat blacked[2];
			
			blacked[0] = undistortedImage.clone();

			cv::Rect roi_rect(0, 0, undistortedImage.size().width/2, undistortedImage.size().height);
			cv::Mat roiLeft(blacked[0], roi_rect);
			roiLeft.setTo(Scalar(0));


			//cv::Mat leftRoiBlack(blacked[0], cv::Rect(0, 0, cameraImageSize.width / 2, cameraImageSize.height));
			//{
			//	cv::Mat &black = blacked[0];
			//	black(leftRoiBlack) = cv::Mat(leftRoiBlack.size(), CV_8UC3);
			//}

			blacked[1] = undistortedImage.clone();
			roi_rect = cv::Rect(undistortedImage.size().width / 2, 0, undistortedImage.size().width / 2, undistortedImage.size().height);
			cv::Mat roiRight(blacked[1], roi_rect);
			roiRight.setTo(Scalar(0));
			//cv::Mat rightRoiBlack(blacked[1], cv::Rect(cameraImageSize.width / 2, 0, cameraImageSize.width / 2, cameraImageSize.height));
			//{
			//	cv::Mat &black = blacked[1];
			//	black(rightRoiBlack).setTo(Scalar(0));
			//}

			// find both checkerboards
			for (int i = 0; i < 2; i++)
			{
				// detect checkerboard
				vector<Point2f> pointbuf;
				bool found = found = findChessboardCorners(blacked[i], c.boardSize, pointbuf,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
			
				// improve the found corners' coordinate accuracy
				bool refinement = true;
				if (refinement)
				{
					Mat viewGray;
					cvtColor(blacked[i], viewGray, CV_BGR2GRAY);
					if (c.pattern == CHESSBOARD && found) cornerSubPix(viewGray, pointbuf, Size(11, 11),
						Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
				}

				if (found)
				{
					drawChessboardCorners(undistortedImage, c.boardSize, Mat(pointbuf), found);
								
					computeExtrinsics(pointbuf, c, c.boardSize, c.squareSize, rotationVector[i], translationVector[i]);

					drawAxis(undistortedImage, c.cameraMatrix, c.distortionCoefficient, rotationVector[i], translationVector[i], 2 * c.squareSize);
				}
			}

			// estimate transform between both

			// draw distance line
		}

		// show image
		cv::imshow(windowName, undistortedImage);

        // quite program on keyboard input
		char key = cvWaitKey(1);
		if ((key & 255) == 27)
			break;
    }

	std::cout << "\nCamera loop stopped.\n\n";
	std::cout << "Disconnecting camera ...\n";
	camera.release();
    cv::destroyAllWindows();
	std::cout << "done.\n\nRegular program exit.\n" << std::endl;

    return 0;
}

