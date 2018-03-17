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

void createCombinedImage(cv::Mat &original, cv::Mat &undistorted, cv::Mat &combined, float imageResizeFactor)
{

	cv::Mat resizedOriginalImg;
	cv::resize(original, resizedOriginalImg, cv::Size(original.size().width * imageResizeFactor, original.size().height * imageResizeFactor));
	int resOrigWidth = resizedOriginalImg.size().width;
	int resOrigHeight = resizedOriginalImg.size().height;

	cv::Mat resizedUndistortedImg;
	float resizeFactorWidth = (float)undistorted.size().width / (float)resizedOriginalImg.size().width;
	float resizeFactorHeight = (float)undistorted.size().height / (float)resizedOriginalImg.size().height;
	float undistortedImageResizeFactor = 1.f / std::floor((std::max(resizeFactorWidth, resizeFactorHeight)));
	cv::resize(undistorted, resizedUndistortedImg, cv::Size(undistorted.size().width * undistortedImageResizeFactor, undistorted.size().height * undistortedImageResizeFactor));
	int resUndistortedWidth = resizedUndistortedImg.size().width;
	int resUndistortedHeight = resizedUndistortedImg.size().height;


	combined = cv::Mat(max(resOrigHeight, resUndistortedHeight), resOrigWidth + resUndistortedWidth, CV_8UC3);

	cv::Mat roi_original = combined(cv::Rect(0, 0, resOrigWidth, resOrigHeight));
	resizedOriginalImg.copyTo(roi_original);
	cv::Mat roi_undistorted = combined(cv::Rect(resOrigWidth, 0, resUndistortedWidth, resUndistortedHeight));
	resizedUndistortedImg.copyTo(roi_undistorted);
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
	
	const char* calibrationFilename;
	const char* imageListFilename = 0;

	int i, nframes = 10;
	bool writeExtrinsics = false, writePoints = false;
	bool undistortImage = false;
	bool performUndistortionOnly = false;

	int flags = 0;

	bool flipVertical = false;
	//bool showUndistorted = true;
	bool showOutput = false;


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
		else if (strcmp(s, "-c") == 0)
		{
			calibrationFilename = argv[++i];
		}
		else if (strcmp(s, "-i") == 0)
		{
			imageListFilename = argv[++i];
		}
		else if (strcmp(s, "-showoutput") == 0)
		{
			showOutput = true;
		}
		//else if (s[0] != '-')
		//{
		//	if (isdigit(s[0]))
		//		sscanf(s, "%d", &cameraId);
		//	else
		//		imageListFilename = s;
		//}
		else
			return fprintf(stderr, "Unknown option %s", s), -1;
	}

	if (imageListFilename)
	{
		if (readStringList(imageListFilename, imageList))
		{
			printf("Using image list : '%s' as input\n", imageListFilename);
			mode = CAPTURING;
		}
		else
			return fprintf(stderr, "Could not initialize file list '%s'\n", imageListFilename), -2;
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

	{
		// load calibration from file
		printf("loading calibration file\n");

		if (!loadCameraParams(std::string(calibrationFilename), c))
		{
			printf("ERROR: Could not read calibration file '%s'\n", calibrationFilename);
			return -1;
		}

		mode = CALIBRATED;



	}


	if (mode == CALIBRATED)
	{
		int visualizationMode = 1; // 0 - draw checkerboard, 1 - draw degrees, 2 = none

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

			//cv::Mat resized;
			//int resizeFactor = 4;
			//resize(view, resized, cv::Size(view.size().width / resizeFactor, view.size().height / resizeFactor));
			//view = resized;

			c.imageSize = view.size();
			

			Mat temp = view.clone();
			Mat undistortedImage;

			Mat updateCameraMatrix;

			imageSize = temp.size();
			
			cv::Size updatedImageSize = imageSize;

			double fovH, fovV;
			computeFov(c.cameraMatrix, imageSize, fovH, fovV);
			
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
					if (drawFov > 90)
						break;

					double drawFovRad = drawFov / 180.0 * CV_PI;
				
					// rotate a point about angle alpha
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
			
			//undistortedImage = temp;
			remap(temp, undistortedImage, map1, map2, INTER_LINEAR, BORDER_CONSTANT);
			
			printf("%d/%d Undistorted\n", (int)imagePoints.size(), nframes);
			printf("image size %d x %d\n", undistortedImage.size().width, undistortedImage.size().height);

			// create combined image


			char keypress = 0;


			cv::Mat combined;
			createCombinedImage(view, undistortedImage, combined, 0.25f);

			if (showOutput)
			{
				float resizeFactor;
				showImage(combined, std::string("Image View"), keypress, resizeFactor);

			}
			
			
			keypress = cvWaitKey(1);

			char filename[1024];
			//sprintf(filename, "%s_undistorted.jpg", imageList[i].c_str());
			//printf("writing file %s\n", filename);
			//imwrite(filename, undistortedImage);

			sprintf(filename, "%s_combined.jpg", imageList[i].c_str());
			printf("writing file %s\n", filename);
			imwrite(filename, combined);


			int c = 0xff & (int)keypress;
			if ((c & 255) == 27 || c == 'q' || c == 'Q')
				break;

			// advance frames
			//if (c == 'n')
			{
				i++;
				if (i == (int)imageList.size())
					break;
			}
			//if (c == 'u')
			//{
			//	showUndistorted = !showUndistorted;
			//}
			if (c == 'v')
			{
				visualizationMode++;
				visualizationMode = visualizationMode % 3;
			}
			if (c == 's')
			{
				showOutput = !showOutput;
			}


		}
	}

	return 0;
}

