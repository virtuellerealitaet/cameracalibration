

#include <iostream>
#include <vector>

#include <Windows.h>

// OpenCV
#include <opencv2/opencv.hpp>

SHORT WINAPI GetAsyncKeyState(
	_In_ int vKey
);

int main(int argc, char *argv[])
{

	//std::string outputPrefix = std::string("default");
	char *outputPrefix = "default";
	int cameraId = 0;

	int camera_width, camera_height;

	//camera_width = 1920;
	//camera_height = 1080;

	camera_width = 640;
	camera_height = 480;

	std::cout << outputPrefix << std::endl;

	if (argc > 1)
	{
		for (int i = 1; i < argc; i++)
		{
			const char* s = argv[i];

			if (strcmp(s, "-id") == 0) // camera id
			{
				if (sscanf(argv[++i], "%u", &cameraId) < 0)
					return printf("Invalid camera id\n"), -1;
			}
			else if (strcmp(s, "-o") == 0) // output prefix
			{
				outputPrefix = argv[++i];
			}
			else if (strcmp(s, "-w") == 0) // camera width
			{
				if (sscanf(argv[++i], "%u", &camera_width) <= 0)
					return printf("Invalid resolution width\n"), -1;
			}
			else if (strcmp(s, "-h") == 0) // camera height
			{
				if (sscanf(argv[++i], "%u", &camera_height) <= 0)
					return printf("Invalid resolution height\n"), -1;
			}
			else
				return fprintf(stderr, "Unknown option %s", s), -1;
		}
	}


	
	cv::VideoCapture camera(cameraId);



	camera.set(CV_CAP_PROP_FRAME_WIDTH, camera_width);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, camera_height);

	camera_width = (int)camera.get(CV_CAP_PROP_FRAME_WIDTH);
	camera_height = (int)camera.get(CV_CAP_PROP_FRAME_HEIGHT);

	printf("camera resolution : %d x %d\n", camera_width, camera_height);

	if (!camera.isOpened())
	{
		printf("ERROR: Could not open OpenCV camera\n");
	}
	
    std::string windowName("camera");
	cv::namedWindow(windowName, 1);

	std::cout << "Begin camera loop (press key to stop process)" << std::endl;
		
	cv::Mat view = cv::Mat(cv::Size(camera_width, camera_height), CV_8UC3);

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

		if (GetAsyncKeyState(VK_SPACE) & 0x8000)
		{
			std::string filename = outputPrefix;
			filename.append(std::to_string(frameNumber));
			filename.append(".png");
			frameNumber++;
			Sleep(500);

			cv::Mat copy;
			view.copyTo(copy);

			cv::imwrite(filename, copy);
			printf("%s\n", filename);

			bitwise_not(view, view);
		}

		// save images
		cv::imshow(windowName,view);

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

