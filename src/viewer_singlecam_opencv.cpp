

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

	std::string outputPrefix = std::string("default");
	for (int i = 0; i < argc; i++)
	{
		std::cout << i << " " << std::string(argv[i]).c_str() << std::endl;
	}
	if (argc > 1)
	{
		outputPrefix = std::string(argv[1]);
	}

	std::cout << outputPrefix << std::endl;

	int cameraId = 0;
	cv::VideoCapture camera(0);

	int camera_width, camera_height;
	camera_width = 1920;
	camera_height = 1080;

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

