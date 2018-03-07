

#include <iostream>
#include <vector>

#include <Windows.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// GLM
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/rotate_vector.hpp>

using namespace cv;
using namespace std;

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
);

enum Pattern { CHESSBOARD };

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

struct Pose
{
	cv::Mat rodrigesVector;
	cv::Mat translationVector;
	cv::Mat rotMatrix;

	vector<Point2f> pointbuf;

	
	glm::vec3 translationVectorGLM;
	glm::mat3 rotationMatrixGLM;
	glm::mat4 transformGLM;

	bool found = false; // checkerboard found

	void convertOpenCVtoGLM()
	{
		// convert location of checkerboard (translation)
		translationVectorGLM = glm::vec3(translationVector.at<double>(0), translationVector.at<double>(1), translationVector.at<double>(2));

		// convert OpenCV rotation matrix to GLM matrix
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				rotationMatrixGLM[i][j] = rotMatrix.at<double>(j, i);

		// transformation matrix containing rotation and translation
		transformGLM = glm::translate(glm::mat4(1.f), translationVectorGLM) * glm::mat4(rotationMatrixGLM);
		
	}

};

void improveCheckerboardAccuracy(cv::Mat &img, vector<Point2f> &cornerPoints, cv::Size sz = cv::Size(11,11))
{
	Mat viewGray;
	cvtColor(img, viewGray, CV_BGR2GRAY);
	cornerSubPix(viewGray, cornerPoints, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
	corners.resize(0);

	switch (patternType)
	{
	case CHESSBOARD:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float(j*squareSize),
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

void drawAxis(cv::Mat &_image, cv::Mat _cameraMatrix, cv::Mat _distCoeffs,
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
	line(_image, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 1.5);
	line(_image, imagePoints[0], imagePoints[2], Scalar(0, 255, 0), 1.5);
	line(_image, imagePoints[0], imagePoints[3], Scalar(255, 0, 0), 1.5);

	imagePoints.clear();

	// draw cube

	vector< Point3f > points;
	imagePoints.clear();
	points.clear();

	//// draw ground floor in green
	points.push_back(Point3f(0, 0, 0));
	points.push_back(Point3f(length, 0, 0));
	points.push_back(Point3f(length, length, 0));
	points.push_back(Point3f(0, length, 0));
	projectPoints(points, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);
	vector< Point > imgPoints;
	imgPoints.push_back(imagePoints[0]);
	imgPoints.push_back(imagePoints[1]);
	imgPoints.push_back(imagePoints[2]);
	imgPoints.push_back(imagePoints[3]);
	int lineType = 8;
	const cv::Point *pts = (const cv::Point*) imgPoints.data();
	int npts = 4;
	fillPoly(_image, &pts, &npts, 1, Scalar(0, 255, 0), lineType);

	float lineThickness = 2.f;
	float v = length;

	// draw pillars in blue
	imagePoints.clear();
	points.clear();

	points.push_back(Point3f(0, 0, 0));
	points.push_back(Point3f(0, 0, -v));
	points.push_back(Point3f(v, 0, 0));
	points.push_back(Point3f(v, 0, -v));
	points.push_back(Point3f(v, v, 0));
	points.push_back(Point3f(v, v, -v));
	points.push_back(Point3f(0, v, 0));
	points.push_back(Point3f(0, v, -v));
	projectPoints(points, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);
	line(_image, imagePoints[0], imagePoints[1], cv::Scalar(255, 0, 0), lineThickness);
	line(_image, imagePoints[2], imagePoints[3], cv::Scalar(255, 0, 0), lineThickness);
	line(_image, imagePoints[4], imagePoints[5], cv::Scalar(255, 0, 0), lineThickness);
	line(_image, imagePoints[6], imagePoints[7], cv::Scalar(255, 0, 0), lineThickness);
	
	

	// draw top layer in red
	imagePoints.clear();
	points.clear();

	points.push_back(Point3f(0, 0, -v));
	points.push_back(Point3f(0, v, -v));
	points.push_back(Point3f(v, v, -v));
	points.push_back(Point3f(v, 0, -v));
	projectPoints(points, _rvec, _tvec, _cameraMatrix, _distCoeffs, imagePoints);
	line(_image, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), lineThickness);
	line(_image, imagePoints[1], imagePoints[2], cv::Scalar(0, 0, 255), lineThickness);
	line(_image, imagePoints[2], imagePoints[3], cv::Scalar(0, 0, 255), lineThickness);
	line(_image, imagePoints[3], imagePoints[0], cv::Scalar(0, 0, 255), lineThickness);


	//cv::String msg = format("world origin X %.2f, Y %.2f, Z %.2f", _tvec.at<double>(0), _tvec.at<double>(1), _tvec.at<double>(2));
	//cv::Point textOrigin = cv::Point(imagePoints[0]) + cv::Point(0, 0);
	//
	//int baseLine = 0;
	//Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);

	//// black background behind text
	//int x, y, w, h;
	//x = std::max(0,textOrigin.x);
	//y = std::max(0, -10 + textOrigin.y);
	//w = std::max(0, textSize.width);
	//h = std::max(0, 20);

	//w = std::min(_image.cols-x, w);
	//h = std::min(_image.rows-y, h);
	//	
	//cv::Rect roi_rect(x, y, w, h);

	//{
	//	cv::Mat roi(_image, roi_rect);
	//	roi.setTo(Scalar(0));
	//	putText(_image, msg, textOrigin, 1, 1, Scalar(0, 255, 0));
	//}

	
}

void computeExtrinsics(Pose &pose, CameraCalibration &calib, cv::Size &boardSize, float squareSize)
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
	undistortPoints(pose.pointbuf, imagePoints, calib.cameraMatrix, calib.distortionCoefficient);

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
	pose.rotMatrix = R;

	Rodrigues(R, pose.rodrigesVector);
	pose.translationVector = tvec;

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
	int cameraId = 1;
	cv::VideoCapture camera(cameraId);
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

	int trackingMode = 0; // 0 = single tracking mode, 1 = dual tracking mode

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
				break;
		}

		cv::Mat undistortedImage = view.clone();
		cv::remap(view, undistortedImage, mapx, mapy, INTER_LINEAR);

		if (trackingMode == 0)
		{
			// detect checkerboard
			Pose cb;
			cb.found = findChessboardCorners(undistortedImage, c.boardSize, cb.pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
								
			if (cb.found)
			{
				// improve the found corners' coordinate accuracy
				bool refinement = true;
				if (refinement)
					improveCheckerboardAccuracy(undistortedImage, cb.pointbuf);

				//drawChessboardCorners(undistortedImage, c.boardSize, Mat(pointbuf), found);
				computeExtrinsics(cb, c, c.boardSize, c.squareSize);
				drawAxis(undistortedImage, c.cameraMatrix, c.distortionCoefficient, cb.rodrigesVector, cb.translationVector, 5 * c.squareSize);
			}
		}
		else if (trackingMode == 1)
		{

			if (undistortedImage.empty())
				continue;

			Pose cbPose[2];
			
			cv::Mat blacked[2];
			blacked[0] = undistortedImage.clone();
			blacked[1] = undistortedImage.clone();
			cv::Rect roi_rect(0, 0, undistortedImage.size().width/2, undistortedImage.size().height);
			cv::Mat roiLeft(blacked[0], roi_rect);
			roiLeft.setTo(Scalar(0));
			roi_rect = cv::Rect(undistortedImage.size().width / 2, 0, undistortedImage.size().width / 2, undistortedImage.size().height);
			cv::Mat roiRight(blacked[1], roi_rect);
			roiRight.setTo(Scalar(0));

			// find both checkerboards
			for (int i = 0; i < 2; i++)
			{
				// detect checkerboard
				cbPose[i].found = findChessboardCorners(blacked[i], c.boardSize, cbPose[i].pointbuf,CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

				if (cbPose[i].found)
				{
					// improve the found corners' coordinate accuracy
					bool refinement = true;
					if (refinement)
						improveCheckerboardAccuracy(blacked[i], cbPose[i].pointbuf, Size(11, 11));

					//drawChessboardCorners(undistortedImage, c.boardSize, Mat(pointbuf), found);
					computeExtrinsics(cbPose[i], c, c.boardSize, c.squareSize);
					drawAxis(undistortedImage, c.cameraMatrix, c.distortionCoefficient, cbPose[i].rodrigesVector, cbPose[i].translationVector, 5 * c.squareSize);
				}
			}
			

			if (cbPose[0].found && cbPose[1].found)
			{

				cbPose[0].convertOpenCVtoGLM();
				cbPose[1].convertOpenCVtoGLM();
				
			}
		}
		else if (trackingMode == 2)
		{
						
			const int numCheckerboards = 3;
			static Rect2d checkerboardRoi[numCheckerboards];
			static bool allRoisSet = false;

			// created user-selected regions of interest
			char key = cvWaitKey(1);
			if ((key & 255) == 52)
			{
				allRoisSet = true;
				bool fromCenter = false;
				for (int i = 0; i < numCheckerboards; i++)
				{
					printf("select roi %d\n",i);
					checkerboardRoi[i] = selectROI(windowName, undistortedImage, fromCenter);
					if (checkerboardRoi[i].empty())
					{
						allRoisSet = false;
						break;
					}
					else
						printf("roi %d set\n", i);
				}
			}

			if (allRoisSet)
			{
				Pose cbPose[numCheckerboards];

				// find checkerboards
				for (int i = 0; i < numCheckerboards; i++)
				{
					// create region of interest
					cv::Mat roiImg(undistortedImage, checkerboardRoi[i]);

					// detect checkerboard
					cbPose[i].found = findChessboardCorners(roiImg, c.boardSize, cbPose[i].pointbuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

					if (cbPose[i].found)
					{

						// improve the found corners' coordinate accuracy
						bool refinement = true;
						if (refinement)
							improveCheckerboardAccuracy(roiImg, cbPose[i].pointbuf, cv::Size(11, 11));

						// add offset from roi position
						cv::Point2f offset(checkerboardRoi[i].x, checkerboardRoi[i].y);
						for (auto &p : cbPose[i].pointbuf)
							p = p + offset;

						drawChessboardCorners(undistortedImage, c.boardSize, Mat(cbPose[i].pointbuf), cbPose[i].found);

						computeExtrinsics(cbPose[i], c, c.boardSize, c.squareSize);

						drawAxis(undistortedImage, c.cameraMatrix, c.distortionCoefficient, cbPose[i].rodrigesVector, cbPose[i].translationVector, 5 * c.squareSize);

						cbPose[i].convertOpenCVtoGLM();
					}
				}
				
				if (cbPose[0].found && cbPose[1].found)
				{
					// compute angle from checkerboard A to checkerboard B
					glm::vec3 eulerAtoB = glm::degrees(glm::eulerAngles(glm::quat(glm::inverse(cbPose[0].rotationMatrixGLM) * cbPose[1].rotationMatrixGLM)));
					printf("a_to_B_x = %04.2f a_to_B_x = %04.2f a_to_B_x = %04.2f\n", eulerAtoB.x, eulerAtoB.y, eulerAtoB.z);

					// draw line from checkerboard origin A to checkerboard origin B

					// distance of checkerboards
					glm::vec3 vectorAtoB = cbPose[1].translationVectorGLM - cbPose[0].translationVectorGLM;
					printf("distance A to B = %.2f\n", glm::length(vectorAtoB));

					// origin of A
					glm::vec4 v = glm::vec4(glm::vec3(0, 0, 0), 1.f);
					glm::vec3 frameBinA = glm::inverse(cbPose[1].transformGLM) * cbPose[0].transformGLM * v;

					// project point from A to B
					vector< Point3f > axisPoints;
					axisPoints.push_back(Point3f(0, 0, 0));
					axisPoints.push_back(Point3f(frameBinA.x, frameBinA.y, frameBinA.z));

					vector< Point2f > imagePoints;
					projectPoints(axisPoints, cbPose[1].rodrigesVector, cbPose[1].translationVector, c.cameraMatrix, c.distortionCoefficient, imagePoints);
					line(undistortedImage, imagePoints[0], imagePoints[1], Scalar(0, 0, 255), 1);
					circle(undistortedImage, imagePoints[1], 5, Scalar(0, 255, 255), 3);

				}

				// draw regions of interests for different checkerboards
				rectangle(undistortedImage, checkerboardRoi[0], cv::Scalar(255, 0, 0), 2);
				rectangle(undistortedImage, checkerboardRoi[1], cv::Scalar(0, 255, 0), 2);
				rectangle(undistortedImage, checkerboardRoi[2], cv::Scalar(0, 0, 255), 2);
			}



			// find checkerboard in selected regions

		}
		

		// show image
		cv::imshow(windowName, undistortedImage);

		// quite program on keyboard input
		char key = cvWaitKey(1);
		
		if ((key & 255) == 49)
		{
			trackingMode = 0;
			printf("Single checkerboard tracking\n");
		}
		else if ((key & 255) == 50)
		{
			trackingMode = 1;
			printf("Dual checkerboard tracking\n");
		}
		else if ((key & 255) == 51)
		{
			trackingMode = 2;
			printf("User-selected ROI\n");
		}
				

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

