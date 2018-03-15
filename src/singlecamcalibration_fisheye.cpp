/****************************************************************************************
* Application :	Single Camera Calibration Application using
*
*					-OpenCV3 (http://opencv.org/) and
*					-PS3EYEDriver C API Interface (by Thomas Perl) for Windows build
*					-Video4Linux2 for Linux build
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
***************************************************************************************/

// OpenCV
#include <opencv2/opencv.hpp>

#undef min
#undef max
#include <algorithm>

using namespace cv;
using namespace std;

const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   singlecamcalibration.exe -w 9 -h 6 -pt chessboard -n 15 -d 2000 -o mycam -op -oe\n"
" \n"
" example command line for calibration from a list of stored images:\n"
"   imagelist_creator image_list.xml *.png\n"
"   calibration -w 4 -h 5 -s 0.025 -o camera.yml -op -oe image_list.xml\n"
" where image_list.xml is the standard OpenCV XML/YAML\n"
" use imagelist_creator to create the xml or yaml list\n"
" file consisting of the list of strings, e.g.:\n"
" \n"
"<?xml version=\"1.0\"?>\n"
"<opencv_storage>\n"
"<images>\n"
"view000.png\n"
"view001.png\n"
"<!-- view002.png -->\n"
"view003.png\n"
"view010.png\n"
"one_extra_view.jpg\n"
"</images>\n"
"</opencv_storage>\n";




const char* liveCaptureHelp =
"When the live video from camera is used as input, the following hot-keys may be used:\n"
"  <ESC>, 'q' - quit the program\n"
"  'g' - start capturing images\n"
"  'u' - switch undistortion on/off\n";

static void help()
{
	printf("Single camera calibration using OpenCV\n"
		"Usage:\n"
		"     -useSonyEye              # use sony eye camera instead of default camera\n"
		"     -w <board_width>         # the number of inner corners per one of board dimension\n"
		"     -h <board_height>        # the number of inner corners per another board dimension\n"
		"     [-pt <pattern>]          # the type of pattern: chessboard or circles' grid\n"
		"     [-n <number_of_frames>]  # the number of frames to use for calibration\n"
		"                              # (if not specified, it will be set to the number\n"
		"                              #  of board views actually available)\n"
		"     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
		"                              # (used only for video capturing)\n"
		"     [-s <squareSize>]       # square size in some user-defined units (1 by default)\n"
		"     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
		"     [-op]                    # write detected feature points\n"
		"     [-oe]                    # write extrinsic parameters\n"
		"     [-zt]                    # assume zero tangential distortion\n"
		"     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
		"     [-p]                     # fix the principal point at the center\n"
		"     [-v]                     # flip the captured images around the horizontal axis\n"
		"     [-V]                     # use a video file, and not an image list, uses\n"
		"                              # [input_data] string for the video file name\n"
		"     [-su]                    # show undistorted images after calibration\n"
		"     [-uo]                    # undistortion only, use precalibration from calibration file\n"
		"     [input_data]             # input data, one of the following:\n"
		"                              #  - text file with a list of the images of the board\n"
		"                              #    the text file can be generated with imagelist_creator\n"
		"                              #  - name of video file with a video of the board\n"
		"                              # if input_data not specified, a live view from the camera is used\n"
		"\n");
	printf("\n%s", usage);
	printf("\n%s", liveCaptureHelp);
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

static double computeReprojectionErrors(
	const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

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

std::vector<cv::Point3f> Create3DChessboardCorners(cv::Size boardSize, float squareSize)
{
	// This function creates the 3D points of your chessboard in its own coordinate system
	float width = (boardSize.width - 1)*squareSize;
	float height = (boardSize.height - 1)*squareSize;


	std::vector<cv::Point3f> corners;

	for (int i = 0; i < boardSize.height; i++)
	{
		for (int j = 0; j < boardSize.width; j++)
		{
			corners.push_back(cv::Point3f(float(j*squareSize) - width, float(i*squareSize) - height, 0));
		}
	}

	return corners;
}

static bool runCalibration(vector<vector<Point2f> > imagePoints,
	Size imageSize, Size boardSize, Pattern patternType,
	float squareSize, float aspectRatio,
	int flags, Mat& cameraMatrix, Mat& distCoeffs,
	vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs,
	double& totalAvgErr)
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CV_CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = aspectRatio;

	//distCoeffs = Mat::zeros(8, 1, CV_64F);

	distCoeffs = Mat::zeros(4, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);
	//objectPoints[0] = Create3DChessboardCorners(boardSize, squareSize);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	int flag = 0;
	//flag |= cv::fisheye::CALIB_USE_INTRINSIC_GUESS;
	//flag |= cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC;
	//flag |= cv::fisheye::CALIB_CHECK_COND;
	//flag |= cv::fisheye::CALIB_FIX_SKEW;
	
	flag |= cv::fisheye::CALIB_FIX_K1;
	flag |= cv::fisheye::CALIB_FIX_K2;
	flag |= cv::fisheye::CALIB_FIX_K3;
	flag |= cv::fisheye::CALIB_FIX_K4;
	//flag |= cv::fisheye::CALIB_FIX_INTRINSIC;
	flag |= cv::fisheye::CALIB_FIX_PRINCIPAL_POINT;


	double rms = fisheye::calibrate(objectPoints, imagePoints, imageSize,
		cameraMatrix, // K
		distCoeffs, // D
		rvecs,
		tvecs,
		flag,
		cv::TermCriteria(3, 20, 1e-6)
		);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}


static void saveCameraParams(const string& filename,
	Size imageSize, Size boardSize,
	float squareSize, float aspectRatio, int flags,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const vector<float>& reprojErrs,
	const vector<vector<Point2f> >& imagePoints,
	double totalAvgErr)
{
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	if (flags & CV_CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0)
	{
		sprintf(buf, "flags: %s%s%s%s",
			flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
		cvWriteComment(*fs, buf, 0);
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		cvWriteComment(*fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0);
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
}

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

static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}


static bool runAndSave(const string& outputFilename,
	const vector<vector<Point2f> >& imagePoints,
	Size imageSize, Size boardSize, Pattern patternType, float squareSize,
	float aspectRatio, int flags, Mat& cameraMatrix,
	Mat& distCoeffs, bool writeExtrinsics, bool writePoints)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	printf("Performing calibration using %d detected checkerboards in images of size %d x %d. Checkerboard square size %.5f mm\n", imagePoints.size(), imageSize.width, imageSize.height, squareSize);

	bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
		aspectRatio, flags, cameraMatrix, distCoeffs,
		rvecs, tvecs, reprojErrs, totalAvgErr);
	printf("%s. avg reprojection error = %.2f\n",
		ok ? "Calibration succeeded" : "Calibration failed",
		totalAvgErr);

	if (ok)
		saveCameraParams(outputFilename, imageSize,
			boardSize, squareSize, aspectRatio,
			flags, cameraMatrix, distCoeffs,
			writeExtrinsics ? rvecs : vector<Mat>(),
			writeExtrinsics ? tvecs : vector<Mat>(),
			writeExtrinsics ? reprojErrs : vector<float>(),
			writePoints ? imagePoints : vector<vector<Point2f> >(),
			totalAvgErr);
	return ok;
}

void showImage(cv::Mat &img, std::string &windowName, char &key, float &imageResizeFactor)
{
	// show image
	// if image is small enough show it
	if (img.size().width <= 800 && img.size().height <= 600)
	{
		imageResizeFactor = 1.f;
		cv::imshow(windowName, img);
	}
	else
	{
		float resizeFactorWidth = (float)img.size().width / (float)800;
		float resizeFactorHeight = (float)img.size().height / (float)600;
		imageResizeFactor = 1.f / std::floor((std::max(resizeFactorWidth, resizeFactorHeight)));
		printf("image resize factor on showimage %.2f\n", imageResizeFactor);

		cv::Mat resizedImg;
		cv::resize(img, resizedImg, cv::Size(img.size().width * imageResizeFactor, img.size().height * imageResizeFactor));
		cv::imshow(windowName, resizedImg);
	}

	key = cvWaitKey(1);
}

static void computeFov(const cv::Mat cameraMatrix, cv::Size imageSize, double &fovH, double &fovV)
{
	double fx, fy; // focal lengths in x and y
	double x, y; // image dimension
	double s; //

	fx = cameraMatrix.at<double>(0, 0);
	fy = cameraMatrix.at<double>(1, 1);
	s = cameraMatrix.at<double>(0, 1);
	x = cameraMatrix.at<double>(0, 2);
	y = cameraMatrix.at<double>(1, 2);

	double ident = cameraMatrix.at<double>(2, 2);

	//	f_x		s		x
	//	0		f_y		y
	//	0		0		1

	printf("fx : %.2f\n", (float)fx);
	printf("fy : %.2f\n", (float)fy);
	printf("s : %.2f\n", (float)s);
	printf("x : %.2f\n", (float)x);
	printf("y : %.2f\n", (float)y);
	printf("ident : %.2f\n", (float)ident);

	double imgWidth = imageSize.width;
	double imgHeight = imageSize.height;


	fovH = 2 * atan(imgWidth / (fx)) * 180.0 / CV_PI; // analog : sensor width / focal length
	fovV = 2 * atan(imgHeight / (fy)) * 180.0 / CV_PI;

	printf("FOV horizontal : %.2f\n", (float)fovH);
	printf("FOV vertical : %.2f\n", (float)fovV);

	printf("principal points : %.2f, %.2f\n", (float)(imgWidth /fx), (float)(imgHeight /fy));

	
}

int main(int argc, char** argv)
{
	Size boardSize, imageSize;
	float squareSize = 1.f, aspectRatio = 1.f;
	//Mat cameraMatrix, distCoeffs;
	const char* outputFilename = "out_camera_data.yml";
	const char* inputFilename = 0;

	int i, nframes = 10;
	bool writeExtrinsics = false, writePoints = false;
	bool undistortImage = false;
	bool performUndistortionOnly = false;

	int flags = 0;

	bool flipVertical = false;
	bool showUndistorted = false;
	bool videofile = false;
	int delay = 1000;
	clock_t prevTimestamp = 0;
	int mode = DETECTION;
	int cameraId = 0;
	vector<vector<Point2f> > imagePoints;
	vector<string> imageList;
	Pattern pattern = CHESSBOARD;


	CameraCalibration c;


	if (argc < 2)
	{
		help();
		return 0;
	}

	for (i = 1; i < argc; i++)
	{
		const char* s = argv[i];
		if (strcmp(s, "-w") == 0)
		{
			if (sscanf(argv[++i], "%u", &boardSize.width) != 1 || boardSize.width <= 0)
				return fprintf(stderr, "Invalid board width\n"), -1;
		}
		else if (strcmp(s, "-h") == 0)
		{
			if (sscanf(argv[++i], "%u", &boardSize.height) != 1 || boardSize.height <= 0)
				return fprintf(stderr, "Invalid board height\n"), -1;
		}
		else if (strcmp(s, "-pt") == 0)
		{
			i++;
			if (!strcmp(argv[i], "circles"))
				pattern = CIRCLES_GRID;
			else if (!strcmp(argv[i], "acircles"))
				pattern = ASYMMETRIC_CIRCLES_GRID;
			else if (!strcmp(argv[i], "chessboard"))
				pattern = CHESSBOARD;
			else
				return fprintf(stderr, "Invalid pattern type: must be chessboard or circles\n"), -1;
		}
		else if (strcmp(s, "-s") == 0)
		{
			if (sscanf(argv[++i], "%f", &squareSize) != 1 || squareSize <= 0)
				return fprintf(stderr, "Invalid board square width\n"), -1;
		}
		else if (strcmp(s, "-n") == 0)
		{
			if (sscanf(argv[++i], "%u", &nframes) != 1 || nframes <= 3)
				return printf("Invalid number of images\n"), -1;
		}
		else if (strcmp(s, "-a") == 0)
		{
			if (sscanf(argv[++i], "%f", &aspectRatio) != 1 || aspectRatio <= 0)
				return printf("Invalid aspect ratio\n"), -1;
			flags |= CV_CALIB_FIX_ASPECT_RATIO;
		}
		else if (strcmp(s, "-d") == 0)
		{
			if (sscanf(argv[++i], "%u", &delay) != 1 || delay <= 0)
				return printf("Invalid delay\n"), -1;
		}
		else if (strcmp(s, "-op") == 0)
		{
			writePoints = true;
		}
		else if (strcmp(s, "-oe") == 0)
		{
			writeExtrinsics = true;
		}
		else if (strcmp(s, "-zt") == 0)
		{
			flags |= CV_CALIB_ZERO_TANGENT_DIST;
		}
		else if (strcmp(s, "-p") == 0)
		{
			flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
		}
		else if (strcmp(s, "-v") == 0)
		{
			flipVertical = true;
		}
		else if (strcmp(s, "-o") == 0)
		{
			outputFilename = argv[++i];
		}
		else if (strcmp(s, "-su") == 0)
		{
			showUndistorted = true;
		}
		else if (strcmp(s, "-uo") == 0)
		{
			performUndistortionOnly = true;
		}
		else if (s[0] != '-')
		{
			if (isdigit(s[0]))
				sscanf(s, "%d", &cameraId);
			else
				inputFilename = s;
		}
		else
			return fprintf(stderr, "Unknown option %s", s), -1;
	}

	if (inputFilename)
	{
		if (readStringList(inputFilename, imageList))
		{
			printf("Using image list : '%s' as input\n", inputFilename);
			mode = CAPTURING;
		}
		else
			return fprintf(stderr, "Could not initialize file list '%s'\n", inputFilename), -2;
	}
	else
		return fprintf(stderr, "No image list given\n"), -2;

	if (!imageList.empty())
	{
		printf("Image list containing %d images\n", imageList.size());
		nframes = (int)imageList.size();
	}
	else
		return fprintf(stderr, "Image list is empty\n"), -2;

	namedWindow("Image View", 1);

	if (!performUndistortionOnly)
	{

		for (i = 0; i < nframes; i++)
		{
			Mat view, viewGray;

			printf("Reading image %s..\n", imageList[i].c_str());
			view = imread(imageList[i], 1);

			if (!view.data)
			{
				continue;
			}

			imageSize = view.size();

			if (flipVertical)
				flip(view, view, 0);

			vector<Point2f> pointbuf;
			cvtColor(view, viewGray, CV_BGR2GRAY);

			bool found;
			switch (pattern)
			{
			case CHESSBOARD:
				found = findChessboardCorners(view, boardSize, pointbuf,
					CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
				break;
			case CIRCLES_GRID:
				found = findCirclesGrid(view, boardSize, pointbuf);
				break;
			case ASYMMETRIC_CIRCLES_GRID:
				found = findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID);
				break;
			default:
				return fprintf(stderr, "Unknown pattern type\n"), -1;
			}

			// improve the found corners' coordinate accuracy
			if (pattern == CHESSBOARD && found) cornerSubPix(viewGray, pointbuf, Size(11, 11),
				Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

			if (mode == CAPTURING && found)
			{
				imagePoints.push_back(pointbuf);
				prevTimestamp = clock();
			}

			if (found)
				drawChessboardCorners(view, boardSize, Mat(pointbuf), found);

			string msg;
			if (mode == CAPTURING)
			{
				msg = format("Input image  %d / %d : %s", i, nframes, imageList[i].c_str());
			}
			else {
				printf("something is wrong\n");
				exit(1);
			}

			int baseLine = 0;
			Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
			Point textOrigin(view.cols - 2 * textSize.width - 10, view.rows - 2 * baseLine - 10);
			putText(view, msg, textOrigin, 1, 1, mode != CALIBRATED ? Scalar(0, 0, 255) : Scalar(0, 255, 0));

			char keypress = 0;
			float resizeFactor;
			showImage(view, std::string("Image View"), keypress, resizeFactor);

			int key = 0xff & (int)keypress;

			if ((key & 255) == 27)
				break;


			//if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
			//{
			//    if( runAndSave(outputFilename, imagePoints, imageSize,
			//               boardSize, pattern, squareSize, aspectRatio,
			//               flags, cameraMatrix, distCoeffs,
			//               writeExtrinsics, writePoints))
			//        mode = CALIBRATED;
			//    else
			//        mode = DETECTION;
			//}
		}

		if (imagePoints.size() > 0)
		{
			if (runAndSave(outputFilename, imagePoints, imageSize,
				boardSize, pattern, squareSize, aspectRatio,
				flags, c.cameraMatrix, c.distortionCoefficient,
				writeExtrinsics, writePoints))
				mode = CALIBRATED;
		}
		else
		{

			return fprintf(stderr, "calibration procedure failed\n"), -2;
		}
	}
	else
	{
		// load calibration from file
		printf("loading calibration file\n");

		if (!loadCameraParams(std::string(outputFilename), c))
		{
			printf("ERROR: Could not read calibration file '%s'\n", outputFilename);
			return -1;
		}

		mode = CALIBRATED;
		showUndistorted = true;
	}

	
	if (showUndistorted)
		printf("showing undistorted\n");

	if (mode == CALIBRATED &&  showUndistorted)
	{
			

		int visualizationMode = 0; // 0 - draw checkerboard, 1 - draw degrees, 2 = none


		Mat view, rview, map1, map2;

		float balance = 1.0; // 0.0 crop to visible pixels, 1.0 maintain all pixels from original image
		float divisor_new_focal_length = 2.0;

		i = 0;

		printf("Input image  %d / %d : %s", i, nframes, imageList[i].c_str());

		//for (i = 0; i < (int)imageList.size(); i++)
		//int i = 0;
		while(true)
		{

			//printf("image %d loaded\n", i);

			view = imread(imageList[i], 1);
			if (!view.data)
				continue;



			Mat temp = view.clone();
			Mat undistortedImage;
			
			


			Mat updateCameraMatrix;

			imageSize = temp.size();
			
			cv::Size updatedImageSize = imageSize;

			double fovH, fovV;
			computeFov(c.cameraMatrix, imageSize, fovH, fovV);

#if 1
			updatedImageSize = imageSize;

			fisheye::estimateNewCameraMatrixForUndistortRectify(
				c.cameraMatrix,
				c.distortionCoefficient,
				temp.size(), // input: set size of distorted image
				Matx33d::eye(), // unused - rectification transform
				updateCameraMatrix, // output: new camera matrix
				balance, // input: 0.0 crop to visible pixels, 1.0 maintain all pixels from original image
				updatedImageSize, // input: set size of undistorted image
				divisor_new_focal_length); // FOV divisor

			printf("updatedImageSize %d x %d\n", updatedImageSize.width, updatedImageSize.height);

			computeFov(updateCameraMatrix, updatedImageSize, fovH, fovV);

			fisheye::initUndistortRectifyMap(
				c.cameraMatrix,
				c.distortionCoefficient,
				Matx33d::eye(),
				updateCameraMatrix,
				updatedImageSize,
				CV_16SC2,
				map1,
				map2);
						
			// undistort image
			undistortedImage = temp;
			
			

			computeFov(updateCameraMatrix, updatedImageSize, fovH, fovV);
#else
			updateCameraMatrix = c.cameraMatrix;
			updatedImageSize = imageSize;
			

			fisheye::initUndistortRectifyMap(
				c.cameraMatrix,
				c.distortionCoefficient,
				Matx33d::eye(),
				c.cameraMatrix,
				updatedImageSize,
				CV_16SC2,
				map1,
				map2);
			cv::remap(temp, undistortedImage, map1, map2, INTER_LINEAR, BORDER_CONSTANT);

#endif

			// draw principial point


			// draw circles of eccentricity


			
			// eccentricities in degrees
			double drawFov = 0;

			double fovX, fovY, focalLength, aspectRatio;
			cv::Point2d principalPoint;

			double sensorSizeX = 6.17; // only valid for gopro camera
			double sensorSizeY = 4.63; // only valid for gopro camera

			calibrationMatrixValues(updateCameraMatrix, updatedImageSize, sensorSizeX, sensorSizeY, fovX, fovY, focalLength, principalPoint, aspectRatio);

			// show center -> project principal point into image
			//cv::Point principalPointImage = cv::Point(updateCameraMatrix.at<double>(0, 2), updateCameraMatrix.at<double>(1, 2));

			cv::Point principalPointImage = cv::Point(principalPoint.x / sensorSizeX * updatedImageSize.width,
				principalPoint.y / sensorSizeY * updatedImageSize.height);
			circle(undistortedImage, principalPointImage, 10, cv::Scalar(0, 0, 255), -1);

			



#if 0

			// compute object points 

			float z = 14.3f;
			float y = 0.0f;

			float limitx = -23.f;
			float x = limitx;

			std::vector<Point3f> eccPoints;


			while (true)
			{

				if (x > abs(limitx))
					break;

				Point3f p(x, y, z);
				eccPoints.push_back(p);

				x += 5.f;

			}

			cv::Affine3d affine = cv::Affine3d::Identity();
			std::vector<Point2f> imgPoints;
			imgPoints.resize(eccPoints.size());
			cv::Mat rvec, tvec;
			rvec = Mat::zeros(cv::Size(1, 3), CV_64F);
			tvec = Mat::zeros(cv::Size(1, 3), CV_64F);
						
			
			cv::fisheye::projectPoints(eccPoints,
				imgPoints,
				affine,
				c.cameraMatrix,
				c.distortionCoefficient);

			std::vector<Point2f> undistortedPoints;
				

			for (int i = 0; i < imgPoints.size(); i++)
			{
				circle(temp, imgPoints[i], (int)5, cv::Scalar(0, 255, 0), 5);

				char msg[1024];
				putText(undistortedImage, cv::format("%.1f", eccPoints[i].x), cv::Point(imgPoints[i].x- 40, imgPoints[i].y +50), 1, 2, Scalar(0, 255, 0), 5);
			}

#endif

			if (visualizationMode == 0) // draw checkerboard
			{
				// compute object points 

				//c.squareSize
				float gridDistance = 150.f;
			
				std::vector<Point3f> gridPoints;

				int numHorPoints = 10;
				int numVertPoints = 7;
				int interpolationDensity = 10;

				for (int x = -numHorPoints/2; x < numHorPoints/2; x++)
				{
					for (int xinterP = 0; xinterP < interpolationDensity; xinterP++)
					{

						for (int y = -numVertPoints/2; y <= numVertPoints/2; y++)
						{
							for (int yinterP = 0; yinterP < interpolationDensity; yinterP++)
							{
						
								float xcoord = ((float)x + ((float)xinterP / interpolationDensity)) * c.squareSize;
								float ycoord = ((float)y + ((float)yinterP / interpolationDensity)) * c.squareSize;

								Point3f p(xcoord, ycoord, gridDistance);
								gridPoints.push_back(p);

								if (xinterP != 0)
									break;
							}
						
						
						}
					}
				}

				cv::Affine3d affine = cv::Affine3d::Identity();
				std::vector<Point2f> imgPoints;
				imgPoints.resize(gridPoints.size());
				cv::Mat rvec, tvec;
				rvec = Mat::zeros(cv::Size(1, 3), CV_64F);
				tvec = Mat::zeros(cv::Size(1, 3), CV_64F);


				cv::fisheye::projectPoints(gridPoints,
					imgPoints,
					affine,
					c.cameraMatrix,
					c.distortionCoefficient);

				for (int i = 0; i < imgPoints.size(); i++)
				{
					circle(temp, imgPoints[i], (int)2, cv::Scalar(0, 255, 0), 5);
					//char msg[1024];
					//putText(undistortedImage, cv::format("%.1f", eccPoints[i].x), cv::Point(imgPoints[i].x - 40, imgPoints[i].y + 50), 1, 2, Scalar(0, 255, 0), 5);
				}
			}


			
			if (visualizationMode == 1) // draw checkerboard
			{
				while (true)
				{
					// compute radius from field of view
					//fovH = 2 * atan(imgWidth / (2 * fx)) * 180.0 / CV_PI;

					//double fx = c.cameraMatrix.at<double>(0, 0);
					//double fx = newK.at<double>(0, 0);

					//float radius = tan(drawFov / (180.0 / CV_PI) / 2) * (2 * fx) / 2;
					//fov = atan(x *0.5 / fx)
					//tan(fov) = tan(atan(x *0.5 / fx))
					//tan(fov) = x *0.5 / fx
					//tan(fov) * fx = x *0.5
					//tan(fov) * fx * 2 = x
				
					//float radius = tan(drawFov / (180.0 / CV_PI) / 2) * (fx) * 2;
					//float radius = tan(drawFov / (180.0 / CV_PI)) * fx * 2;
					//alpha = 2 * atan (d/2f)
				
					//std::cout << "radius : " << radius << std::endl;
				
					if (drawFov > 90)
						break;

					double drawFovRad = drawFov / 180.0 * CV_PI;
				
					// rotate a point about alpha
					cv::Point3f p;
				
					p.z = cos(drawFovRad);
					p.x = sqrt(1 - (p.z*p.z)); // create normalized vector in direction of eccentricity
					p = p * 100;
				


					std::vector<Point3f> eccPoints;
					eccPoints.push_back(p);
							
								
					cv::Affine3d affine = cv::Affine3d::Identity();
					std::vector<Point2f> imgPoints;
					imgPoints.resize(1);
					cv::Mat rvec, tvec;
					rvec = Mat::zeros(cv::Size(1, 3), CV_64F);
					tvec = Mat::zeros(cv::Size(1, 3), CV_64F);

					//cv::fisheye::projectPoints(eccPoints, imgPoints, rvec, tvec, updateCameraMatrix, c.distortionCoefficient);
					cv::fisheye::projectPoints(eccPoints,
						imgPoints,
						affine,
						c.cameraMatrix,
						c.distortionCoefficient);
				
					// compute radius
					cv::Point2f diffFromPrincipalPoint = imgPoints[0] - cv::Point2f(principalPointImage);
					double radius = diffFromPrincipalPoint.x;

					if (radius > 0)
					{
				
						//float radius = fovX;
						circle(temp, principalPointImage, (int)radius, cv::Scalar(0, 255, 0),5);
						char msg[1024];
						putText(temp, cv::format("%d", (int)drawFov), cv::Point(principalPointImage.x + radius, principalPointImage.y), 1, 2, Scalar(0, 255, 0),5);

						circle(temp, imgPoints[0], 10, cv::Scalar(0, 0, 255), -1);

						if (drawFov > 0)
						{
							putText(undistortedImage, cv::format("%d", (int)drawFov), cv::Point(principalPoint.x - radius - 40, principalPoint.y), 1, 2, Scalar(0, 255, 0), 5);
							putText(undistortedImage, cv::format("%d", (int)drawFov), cv::Point(principalPoint.x, principalPoint.y + radius +30), 1, 2, Scalar(0, 255, 0), 5);
							putText(undistortedImage, cv::format("%d", (int)drawFov), cv::Point(principalPoint.x, principalPoint.y - radius), 1, 2, Scalar(0, 255, 0), 5);
						}

					}

					// compute radius
					if (drawFov > fovH)
						break;

					drawFov += 5;
				
				}
			}
			
			if (showUndistorted)
			{
				//undistortedImage = temp;
				remap(temp, undistortedImage, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
			}
			else
				undistortedImage = temp;
			
			
			printf("%d/%d Undistorted\n", (int)imagePoints.size(), nframes);
			printf("image size %d x %d", undistortedImage.size().width, undistortedImage.size().height);

			char keypress = 0;
			float resizeFactor;
			showImage(undistortedImage, std::string("Image View"), keypress, resizeFactor);
			
			keypress = cvWaitKey(0);

			char filename[1024];
			sprintf(filename, "%s_undistorted.jpg", imageList[i].c_str());
			imwrite(filename, undistortedImage);

			int c = 0xff & (int)keypress;
			if ((c & 255) == 27 || c == 'q' || c == 'Q')
				break;
			if (c == 'n')
			{
				i++;
				if (i == (int)imageList.size())
					break;
			}
			if (c == 'u')
			{
				showUndistorted = !showUndistorted;
			}
			if (c == 'v')
			{
				visualizationMode++;
				visualizationMode = visualizationMode % 3;
			}
			if (c == '1')
			{
				balance -= 0.1;
				balance = max(0.f, balance);
			}
			if (c == '2')
			{
				balance += 0.1;
				balance = min(1.f, balance);
			}
			if (c == '3')
			{
				divisor_new_focal_length -= 0.1;
				divisor_new_focal_length = max(0.f, divisor_new_focal_length);
			}
			if (c == '4')
			{
				divisor_new_focal_length += 0.1;
				divisor_new_focal_length = min(3.f, divisor_new_focal_length);
			}

		}
	}

	return 0;
}

