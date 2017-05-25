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

#define CALIBRATION_GUIDED

#ifdef CALIBRATION_GUIDED

#include <stdafx.h>
#include "ThreadClass.h"

#include "circlesgrid.hpp"

using namespace cv;
using namespace std;

const char * usage =
" \nexample command line for calibration from a live feed.\n"
"   calibration  -w 4 -h 5 -s 0.025 -o camera.yml -op -oe\n"
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
    printf( "This is a camera calibration sample.\n"
        "Usage: calibration\n"
        "     -w <board_width>         # the number of inner corners per one of board dimension\n"
        "     -h <board_height>        # the number of inner corners per another board dimension\n"
        "     [-pt <pattern>]          # the type of pattern: chessboard or circles' grid\n"
        "     [-n <number_of_frames>]  # the number of frames to use for calibration\n"
        "                              # (if not specified, it will be set to the number\n"
        "                              #  of board views actually available)\n"
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
        "     [input_data]             # input data, one of the following:\n"
        "                              #  - text file with a list of the images of the board\n"
        "                              #    the text file can be generated with imagelist_creator\n"
        "                              #  - name of video file with a video of the board\n"
        "                              # if input_data not specified, a live view from the camera is used\n"
        "\n" );
    printf("\n%s",usage);
    printf( "\n%s", liveCaptureHelp );
}

enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };

Ptr<SimpleBlobDetector> blobDetector;
SimpleBlobDetector::Params blobParams;

static double computeReprojectionErrors(
        const vector<vector<Point3f> >& objectPoints,
        const vector<vector<Point2f> >& imagePoints,
        const vector<Mat>& rvecs, const vector<Mat>& tvecs,
        const Mat& cameraMatrix, const Mat& distCoeffs,
        vector<float>& perViewErrors )
{
    vector<Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); i++ )
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
                      cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err*err/n);
        totalErr += err*err;
        totalPoints += n;
    }

    return std::sqrt(totalErr/totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
    corners.resize(0);

    switch(patternType)
    {
      case CHESSBOARD:
      case CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float(j*squareSize),
                                          float(i*squareSize), 0));
        break;

      case ASYMMETRIC_CIRCLES_GRID:
        for( int i = 0; i < boardSize.height; i++ )
            for( int j = 0; j < boardSize.width; j++ )
                corners.push_back(Point3f(float((2*j + i % 2)*squareSize),
                                          float(i*squareSize), 0));
        break;

      default:
        CV_Error(CV_StsBadArg, "Unknown pattern type\n");
    }
}

static bool runCalibration( vector<vector<Point2f> > imagePoints,
                    Size imageSize, Size boardSize, Pattern patternType,
                    float squareSize, float aspectRatio,
                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                    vector<float>& reprojErrs,
                    double& totalAvgErr)
{
    cameraMatrix = Mat::eye(3, 3, CV_64F);
    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        cameraMatrix.at<double>(0,0) = aspectRatio;

    distCoeffs = Mat::zeros(8, 1, CV_64F);

    vector<vector<Point3f> > objectPoints(1);
    calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
                    distCoeffs, rvecs, tvecs, flags|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
                    ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    printf("RMS error reported by calibrateCamera: %g\n", rms);

    bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

    return ok;
}


static void saveCameraParams( const string& filename,
                       Size imageSize, Size boardSize,
                       float squareSize, float aspectRatio, int flags,
                       const Mat& cameraMatrix, const Mat& distCoeffs,
                       const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                       const vector<float>& reprojErrs,
                       const vector<vector<Point2f> >& imagePoints,
                       double totalAvgErr )
{
    FileStorage fs( filename, FileStorage::WRITE );

    time_t tt;
    time( &tt );
    struct tm *t2 = localtime( &tt );
    char buf[1024];
    strftime( buf, sizeof(buf)-1, "%c", t2 );

    fs << "calibration_time" << buf;

    if( !rvecs.empty() || !reprojErrs.empty() )
        fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;
    fs << "board_width" << boardSize.width;
    fs << "board_height" << boardSize.height;
    fs << "square_size" << squareSize;

    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
        fs << "aspectRatio" << aspectRatio;

    if( flags != 0 )
    {
        sprintf( buf, "flags: %s%s%s%s",
            flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
            flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
            flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
            flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
        cvWriteComment( *fs, buf, 0 );
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;
    if( !reprojErrs.empty() )
        fs << "per_view_reprojection_errors" << Mat(reprojErrs);

    if( !rvecs.empty() && !tvecs.empty() )
    {
        CV_Assert(rvecs[0].type() == tvecs[0].type());
        Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
        for( int i = 0; i < (int)rvecs.size(); i++ )
        {
            Mat r = bigmat(Range(i, i+1), Range(0,3));
            Mat t = bigmat(Range(i, i+1), Range(3,6));

            CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
            CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
            //*.t() is MatExpr (not Mat) so we can use assignment operator
            r = rvecs[i].t();
            t = tvecs[i].t();
        }
        cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
        fs << "extrinsic_parameters" << bigmat;
    }

    if( !imagePoints.empty() )
    {
        Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
        for( int i = 0; i < (int)imagePoints.size(); i++ )
        {
            Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
            Mat imgpti(imagePoints[i]);
            imgpti.copyTo(r);
        }
        fs << "image_points" << imagePtMat;
    }
}

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


static bool runAndSave(const string& outputFilename,
                const vector<vector<Point2f> >& imagePoints,
                Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                float aspectRatio, int flags, Mat& cameraMatrix,
                Mat& distCoeffs, bool writeExtrinsics, bool writePoints )
{
    vector<Mat> rvecs, tvecs;
    vector<float> reprojErrs;
    double totalAvgErr = 0;

    bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
                   aspectRatio, flags, cameraMatrix, distCoeffs,
                   rvecs, tvecs, reprojErrs, totalAvgErr);
    printf("%s. avg reprojection error = %.2f\n",
           ok ? "Calibration succeeded" : "Calibration failed",
           totalAvgErr);

    if( ok )
        saveCameraParams( outputFilename, imageSize,
                         boardSize, squareSize, aspectRatio,
                         flags, cameraMatrix, distCoeffs,
                         writeExtrinsics ? rvecs : vector<Mat>(),
                         writeExtrinsics ? tvecs : vector<Mat>(),
                         writeExtrinsics ? reprojErrs : vector<float>(),
                         writePoints ? imagePoints : vector<vector<Point2f> >(),
                         totalAvgErr );
    return ok;
}


int minThresholdValue = 70;
int maxThresholdValue = 175;
int filterByArea = true;
int minArea = 176;
int filterByCircularity = 0;
int minCircularity = 10;
int filterByConvexity = 0;
int minConvexity = 87;
int filterByInertia = 0;
int minInertiaRatio = 1;

int method = 0;

static void initBlobDetectorParams()
{

	// Change thresholds
	blobParams.minThreshold = 70;
	blobParams.maxThreshold = 175;

	// Filter by Area.
	blobParams.filterByArea = true;
	blobParams.minArea = 176;

	// Filter by Circularity
	blobParams.filterByCircularity = false;
	blobParams.minCircularity = 0.1;

	// Filter by Convexity
	blobParams.filterByConvexity = false;
	blobParams.minConvexity = 0.87;

	// Filter by Inertia
	blobParams.filterByInertia = false;
	blobParams.minInertiaRatio = 0.01;

}



static void updateBlobDetectorParams()
{
	blobDetector = SimpleBlobDetector::create(blobParams);
}


static void minThresholdTrackbar(int value, void* userparams)
{
	minThresholdValue = value;
	blobParams.minThreshold = float(minThresholdValue);
	cout << "min threshold " << blobParams.minThreshold << endl;
	updateBlobDetectorParams();
}

static void maxThresholdTrackbar(int value, void* userparams)
{
	maxThresholdValue = value;
	blobParams.maxThreshold = float(maxThresholdValue);
	cout << "max threshold " << blobParams.maxThreshold << endl;
	updateBlobDetectorParams();
}

static void filterByAreaTrackbar(int value, void* userparams)
{
	filterByArea = value;
	blobParams.filterByArea = bool(filterByArea);
	cout << "filterByArea " << blobParams.filterByArea << endl;
	updateBlobDetectorParams();
}

static void minAreaTrackbar(int value, void* userparams)
{
	minArea = value;
	blobParams.minArea = float(minArea);
	cout << "minArea " << blobParams.minArea << endl;
	updateBlobDetectorParams();
}

static void filterByCircularityTrackbar(int value, void* userparams)
{
	filterByCircularity = value;
	blobParams.filterByCircularity = bool(filterByCircularity);
	cout << "filterByCircularity " << blobParams.filterByCircularity << endl;
	updateBlobDetectorParams();
}

static void minCircularityTrackbar(int value, void* userparams)
{
	minCircularity = value;
	blobParams.minCircularity = float(minCircularity) / 100;
	cout << "minCircularity " << blobParams.minCircularity << endl;
	updateBlobDetectorParams();
}

static void filterByConvexityTrackbar(int value, void* userparams)
{
	filterByConvexity = value;
	blobParams.filterByConvexity = bool(filterByConvexity);
	cout << "filterByConvexity " << blobParams.filterByConvexity << endl;
	updateBlobDetectorParams();
}

static void minConvexityTrackbar(int value, void* userparams)
{
	minConvexity = value;
	blobParams.minConvexity = float(minConvexity) / 100;
	cout << "minConvexity " << blobParams.minConvexity << endl;
	updateBlobDetectorParams();
}

static void filterByInertiaTrackbar(int value, void* userparams)
{
	filterByInertia = value;
	blobParams.filterByInertia = bool(filterByInertia);
	cout << "filterByInertia " << blobParams.filterByInertia << endl;
	updateBlobDetectorParams();
}

static void minInertiaRatioTrackbar(int value, void* userparams)
{
	minInertiaRatio = value;
	blobParams.minInertiaRatio = float(minInertiaRatio) / 100;
	cout << "minInertiaRatio " << blobParams.minInertiaRatio << endl;
	updateBlobDetectorParams();
}

static bool customFindAsymmCirclesGrid(cv::InputOutputArray mat, cv::Size patternSize, vector<Point2f> &_centers, const Ptr<FeatureDetector> &featureDetect)
{
	std::vector<KeyPoint> keypoints;
	featureDetect.get()->detect(mat, keypoints);
	
	drawKeypoints(mat, keypoints, mat, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);


	std::vector<Point2f> centers;
	centers.clear();

	std::vector<Point2f> points;

	for (size_t i = 0; i < keypoints.size(); i++)
	{
		points.push_back(keypoints[i].pt);
	}

	
	CirclesGridFinderParameters parameters;
	parameters.vertexPenalty = -0.6f;
	parameters.vertexGain = 1;
	parameters.existingVertexGain = 10000;
	parameters.edgeGain = 1;
	parameters.edgePenalty = -0.6f;

	parameters.gridType = CirclesGridFinderParameters::ASYMMETRIC_GRID;
	

	const int attempts = 2;
	const size_t minHomographyPoints = 4;
	Mat H;
	for (int i = 0; i < attempts; i++)
	{
		centers.clear();

		CirclesGridFinder boxFinder(patternSize, points, parameters);
		bool isFound = false;

		

		try
		{
			isFound = boxFinder.findHoles();
		}
		catch (const cv::Exception &)
		{
		}

		//isFound = true;

		if (isFound)
		{
			boxFinder.getAsymmetricHoles(centers);

			if (i != 0)
			{
				Mat orgPointsMat;
				transform(centers, orgPointsMat, H.inv());
				convertPointsFromHomogeneous(orgPointsMat, centers);
			}
			Mat(centers).copyTo(_centers);
			return true;
		}


		boxFinder.getHoles(centers);
		if (i != attempts - 1)
		{
			if (centers.size() < minHomographyPoints)
				break;
			H = CirclesGridFinder::rectifyGrid(boxFinder.getDetectedGridSize(), centers, points, points);
		}

	}

	//featureDetect.get()->detect(mat, keypoints);
	//points.clear();
	//for (size_t i = 0; i < keypoints.size(); i++)
	//{
	//	points.push_back(keypoints[i].pt);
	//}

	_centers = points;

	//Mat(centers).copyTo(_centers);
	return false;

	//if (keypoints.size() == patternSize.height * patternSize.width)
	//	return true;
	//
	//return false;

}

static bool customDrawChessboardCorners(cv::Mat &mat, cv::Size patternSize, vector<Point2f> keypoints, bool patternWasFound)
{

	// draw found points
	for (auto p : keypoints)
		cv::circle(mat, p, 5, cv::Scalar(0, 255, 0), 2);

	int numKeyPoints = keypoints.size();

	std::vector<KeyPoint> _keypoints;
	for (int i = 0; i < numKeyPoints; i++)
	{
		KeyPoint k;
		k.pt = keypoints[i];
		_keypoints.push_back(k);
	}

	drawKeypoints(mat, _keypoints, mat, Scalar(0, 0, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

	return true;
}



int main( int argc, char** argv )
{
    Size boardSize, imageSize;
    float squareSize = 1.f, aspectRatio = 1.f;
    Mat cameraMatrix, distCoeffs;
    const char* outputFilename = "out_camera_data.yml";
    const char* inputFilename = 0;

    int i, nframes = 10;
    bool writeExtrinsics = false, writePoints = false;
    bool undistortImage = false;
    int flags = 0;
    
	VideoCapture capture;
	ThreadCamera *pseye;


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

	bool useEyeCam = false;

	// blob detector	
	initBlobDetectorParams();
	updateBlobDetectorParams();

    if( argc < 2 )
    {
        help();
        return 0;
    }

    for( i = 1; i < argc; i++ )
    {
        const char* s = argv[i];
        if( strcmp( s, "-w" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &boardSize.width ) != 1 || boardSize.width <= 0 )
                return fprintf( stderr, "Invalid board width\n" ), -1;
        }
        else if( strcmp( s, "-h" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &boardSize.height ) != 1 || boardSize.height <= 0 )
                return fprintf( stderr, "Invalid board height\n" ), -1;
        }
        else if( strcmp( s, "-pt" ) == 0 )
        {
            i++;
            if( !strcmp( argv[i], "circles" ) )
                pattern = CIRCLES_GRID;
            else if( !strcmp( argv[i], "acircles" ) )
                pattern = ASYMMETRIC_CIRCLES_GRID;
            else if( !strcmp( argv[i], "chessboard" ) )
                pattern = CHESSBOARD;
            else
                return fprintf( stderr, "Invalid pattern type: must be chessboard or circles\n" ), -1;
        }
        else if( strcmp( s, "-s" ) == 0 )
        {
            if( sscanf( argv[++i], "%f", &squareSize ) != 1 || squareSize <= 0 )
                return fprintf( stderr, "Invalid board square width\n" ), -1;
        }
        else if( strcmp( s, "-n" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &nframes ) != 1 || nframes <= 3 )
                return printf("Invalid number of images\n" ), -1;
        }
        else if( strcmp( s, "-a" ) == 0 )
        {
            if( sscanf( argv[++i], "%f", &aspectRatio ) != 1 || aspectRatio <= 0 )
                return printf("Invalid aspect ratio\n" ), -1;
            flags |= CV_CALIB_FIX_ASPECT_RATIO;
        }
        else if( strcmp( s, "-d" ) == 0 )
        {
            if( sscanf( argv[++i], "%u", &delay ) != 1 || delay <= 0 )
                return printf("Invalid delay\n" ), -1;
        }
        else if( strcmp( s, "-op" ) == 0 )
        {
            writePoints = true;
        }
        else if( strcmp( s, "-oe" ) == 0 )
        {
            writeExtrinsics = true;
        }
        else if( strcmp( s, "-zt" ) == 0 )
        {
            flags |= CV_CALIB_ZERO_TANGENT_DIST;
        }
        else if( strcmp( s, "-p" ) == 0 )
        {
            flags |= CV_CALIB_FIX_PRINCIPAL_POINT;
        }
        else if( strcmp( s, "-v" ) == 0 )
        {
            flipVertical = true;
        }
        else if( strcmp( s, "-V" ) == 0 )
        {
            videofile = true;
        }
        else if( strcmp( s, "-o" ) == 0 )
        {
            outputFilename = argv[++i];
        }
        else if( strcmp( s, "-su" ) == 0 )
        {
            showUndistorted = true;
        }
        else if( s[0] != '-' )
        {
            if( isdigit(s[0]) )
                sscanf(s, "%d", &cameraId);
            else
                inputFilename = s;
        }
		else if (strcmp(s, "-useSonyEye") == 0 )
		{
			useEyeCam = true;
		}
        else
            return fprintf( stderr, "Unknown option %s", s ), -1;
    }

	bool captureIsOpen = false;

    if( inputFilename )
    {
        if( !videofile && readStringList(inputFilename, imageList) )
            mode = CAPTURING;
        else
            capture.open(inputFilename);
    }
	else
	{
		if (useEyeCam)
			pseye = new ThreadCamera();
		else
			capture.open(cameraId);
	}
    
	if (useEyeCam)
	{
		//if (!pseye->initialize(0,320,240,3,60))
		if (!pseye->initialize())
			return fprintf(stderr, "Could not initialize Sony Eye Cam ! \n"), -2;
		else
		{
			pseye->startCapture();
			
			Sleep(500);

			pseye->_autogain = true;
			pseye->_autowhitebalance = true;
			pseye->updateCameraSettings();

			Sleep(500);
		}
	}
	else
	{
		if (!capture.isOpened() && imageList.empty())
			return fprintf(stderr, "Could not initialize video (%d) capture\n", cameraId), -2;
		else
			printf("%s", liveCaptureHelp);
	}

	captureIsOpen = true;


    if( !imageList.empty() )
        nframes = (int)imageList.size();

    namedWindow( "Image View", 1 );

	for (i = 0;; i++)
	{

		int key = 0xff & waitKey(captureIsOpen ? 50 : 500);

		Mat view, viewGray;
		bool blink = false;

		if (useEyeCam)
		{
			Mat view0;
			pseye->receiveFrameCopy(view0);
			view0.copyTo(view);

		}
		else if (capture.isOpened())
		{
			Mat view0;
			capture >> view0;
			view0.copyTo(view);
		}
		else if (i < (int)imageList.size())
		{
			//view = imread(imageList[i], 1);
			cv::Mat origSizeImage = imread(imageList[i], 1);
			
			resize(origSizeImage, view, origSizeImage.size() * 3);
		}
			

		if (!view.data)
		{
			if (imagePoints.size() > 0)
				runAndSave(outputFilename, imagePoints, imageSize,
					boardSize, pattern, squareSize, aspectRatio,
					flags, cameraMatrix, distCoeffs,
					writeExtrinsics, writePoints);
			break;
		}

		imageSize = view.size();

		if (flipVertical)
			flip(view, view, 0);

		vector<Point2f> pointbuf;
		cvtColor(view, viewGray, CV_BGR2GRAY);

		const Ptr<FeatureDetector> &featureDetect = blobDetector;

		// Detect blobs.
		//std::vector<KeyPoint> keypoints;
		//blobDetector.get()->detect(view, keypoints);

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
			if (method == 0)
				found = findCirclesGrid(view, boardSize, pointbuf, CALIB_CB_ASYMMETRIC_GRID, featureDetect);
			else
				found = customFindAsymmCirclesGrid(view, boardSize, pointbuf, featureDetect);
			break;
		default:
			return fprintf(stderr, "Unknown pattern type\n"), -1;
		}
				
		// improve the found corners' coordinate accuracy
		if (pattern == CHESSBOARD && found)
			cornerSubPix(viewGray, pointbuf, Size(11, 11),
				Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

		//if (mode == CAPTURING && found &&
		//	(!captureIsOpen || clock() - prevTimestamp > delay*1e-3*CLOCKS_PER_SEC))
		if (captureIsOpen && key == 'c')
		{
			printf("Calibration points accepted\n");
			imagePoints.push_back(pointbuf);
			prevTimestamp = clock();
			blink = captureIsOpen;
		}
		else if (captureIsOpen && key == 's')
		{
			printf("Image rejected\n");
		}
		else
			i--;

		if (pattern == ASYMMETRIC_CIRCLES_GRID)
			customDrawChessboardCorners(view, boardSize, pointbuf, found);
		//else
		{
			if (found)
				drawChessboardCorners(view, boardSize, Mat(pointbuf), found);
		}



        string msg = mode == CAPTURING ? "100/100" :
            mode == CALIBRATED ? "Calibrated" : "Press 'g' to start";
        int baseLine = 0;
        Size textSize = getTextSize(msg, 1, 1, 1, &baseLine);
        Point textOrigin(view.cols - 2*textSize.width - 10, view.rows - 2*baseLine - 10);

        if( mode == CAPTURING )
        {
            if(undistortImage)
                msg = format( "%d/%d Undist", (int)imagePoints.size(), nframes );
            else
                msg = format( "%d/%d", (int)imagePoints.size(), nframes );
        }

        putText( view, msg, textOrigin, 1, 1,
                 mode != CALIBRATED ? Scalar(0,0,255) : Scalar(0,255,0));

        if( blink )
            bitwise_not(view, view);

        if( mode == CALIBRATED && undistortImage )
        {
            Mat temp = view.clone();
            //undistort(temp, view, cameraMatrix, distCoeffs);

			Mat map1, map2;
			
			initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(), getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0.0), imageSize, CV_16SC2, map1, map2);
			//remap(temp, view, map1, map2, INTER_LINEAR);
			undistort(temp, view, cameraMatrix, distCoeffs, getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1.0));


			//Mat newCamMat;
			//fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize, Matx33d::eye(), newCamMat, 0);
			//fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize, CV_16SC2, map1, map2);

			//fisheye::undistortImage(temp, view, cameraMatrix, distCoeffs);

        }

        cv::imshow("Image View", view);

		
		// create gui for blobe detector
		
		cv::createTrackbar("method", "Image View", &method, 1);

		cv::createTrackbar("minThreshold", "Image View", &minThresholdValue, 500, minThresholdTrackbar);
		cv::createTrackbar("maxThreshold", "Image View", &maxThresholdValue, 500, maxThresholdTrackbar);

		cv::createTrackbar("filterByArea", "Image View", &filterByArea, 1, filterByAreaTrackbar);
		cv::createTrackbar("minArea", "Image View", &minArea, 5000, minAreaTrackbar);

		cv::createTrackbar("filterByCircularity", "Image View", &filterByCircularity, 1, filterByCircularityTrackbar);
		cv::createTrackbar("minCircularity", "Image View", &minCircularity, 100, minCircularityTrackbar);

		cv::createTrackbar("filterByConvexity", "Image View", &filterByConvexity, 1, filterByConvexityTrackbar);
		cv::createTrackbar("minConvexity", "Image View", &minConvexity, 100, minConvexityTrackbar);

		cv::createTrackbar("filterByInertia", "Image View", &filterByInertia, 1, filterByInertiaTrackbar);
		cv::createTrackbar("minInertiaRatio", "Image View", &minInertiaRatio, 100, minInertiaRatioTrackbar);

		

        

        if( (key & 255) == 27 )
            break;

        if( key == 'u' && mode == CALIBRATED )
            undistortImage = !undistortImage;

        if(captureIsOpen && key == 'g' )
        {
            mode = CAPTURING;
            imagePoints.clear();
        }

        if( mode == CAPTURING && imagePoints.size() >= (unsigned)nframes )
        {
            if( runAndSave(outputFilename, imagePoints, imageSize,
                       boardSize, pattern, squareSize, aspectRatio,
                       flags, cameraMatrix, distCoeffs,
                       writeExtrinsics, writePoints))
                mode = CALIBRATED;
            else
                mode = DETECTION;
            if( !captureIsOpen)
                break;
        }
    }

    //if( !captureIsOpen && showUndistorted )
	if (showUndistorted)
    {
        Mat view, rview, map1, map2;
        //initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
        //                        getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1.0, imageSize, 0),
        //                        imageSize, CV_16SC2, map1, map2);

		//Mat newCamMat;
		//fisheye::estimateNewCameraMatrixForUndistortRectify(cameraMatrix, distCoeffs, imageSize, Matx33d::eye(), newCamMat, 0);
		//fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Matx33d::eye(), newCamMat, imageSize, CV_16SC2, map1, map2);


		//Mat temp = view.clone();
		//undistort(temp, view, cameraMatrix, distCoeffs);
		
		//Mat map1, map2;

		initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
			getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 0.0), imageSize, CV_16SC2, map1, map2);


        for( i = 0; i < (int)imageList.size(); i++ )
        {
            // view = imread(imageList[i], 1);

			cv::Mat origSizeImage = imread(imageList[i], 1);
			resize(origSizeImage, view, origSizeImage.size() * 3);


            if(!view.data)
                continue;
            
			undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
            
			

			//remap(view, rview, map1, map2, INTER_LINEAR);

			//fisheye::undistortImage(view, rview, cameraMatrix, distCoeffs);

			//remap(view, rview, map1, map2, INTER_LINEAR);

			//remap(temp, view, map1, map2, INTER_LINEAR);
			//undistort(temp, view, cameraMatrix, distCoeffs, getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1.0));

            imshow("Image View", rview);
            int c = 0xff & waitKey();
            if( (c & 255) == 27 || c == 'q' || c == 'Q' )
                break;
        }
    }

	if (useEyeCam)
	{
		pseye->deinitialize();
		delete pseye;
	}

    return 0;
}


#endif