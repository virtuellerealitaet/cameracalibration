
#include <cambox.h>

#include <iostream>
#include <vector>

#include <Windows.h>

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
	CamBox::VideoCamera* camera = 0;


	unsigned int numPupillabsCameras = CamBox::getNumberAvailablePupilLabsCameras();
	if (numPupillabsCameras == 0)
	{
		printf("ERROR : no pupillabs cameras available. exit.\n");
		return -1;
	}
	else
	{
		printf("%d pupillabs cameras available.\n", numPupillabsCameras);
		if (cameraId >= numPupillabsCameras || cameraId < 0)
		{
			printf("ERROR : no pupillabs camera with camera id = %d available.\n", numPupillabsCameras);
			return -1;
		}
		else
		{
			CamBox::PupilLabsCamera* pupillabscamera = CamBox::createPupilLabsCamera(cameraId, 1920, 1080, 30);
			if (pupillabscamera->initialize())
			{
				// set camera properties ...

				camera = (CamBox::VideoCamera*)pupillabscamera;

				Sleep(500);

				camera->startCapture();

			}
		}
	}
	
    std::string windowName("camera");
	cv::namedWindow(windowName, 1);

	std::cout << "Begin camera loop (press key to stop process)" << std::endl;

	uint8_t  *cameraFrameData = new uint8_t[camera->getFrameMemorySize()];
	cv::Mat view = cv::Mat(cv::Size(camera->getCameraWidth(), camera->getCameraHeight()), CV_8UC3);

	int frameNumber = 0;

    while (true)
    {
		while (true)
		{
			camera->receiveFrameCopy(cameraFrameData);
			view.data = cameraFrameData;
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
				
		//if ((GetAsyncKeyState(VK_SHIFT) & 0x8000) && (GetAsyncKeyState(VK_SPACE) & 0x8000))
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
	camera->deinitialize();

    cv::destroyAllWindows();

	std::cout << "done.\n\nRegular program exit.\n" << std::endl;

    return 0;
}

