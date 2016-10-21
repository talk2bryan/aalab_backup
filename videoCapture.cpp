//VIDEOCAPTURE.CPP

#include <iostream>
#include <opencv2/opencv.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>


int main(int argc, char const *argv[])
{
	cv::VideoCapture cap(0);

	if (!cap.isOpened())
	{
		std::cout << "cam unable to open" <<std::endl;
		exit(-1);
	}

	for (;;)
	{
		cv::Mat frame;
		cap >> frame;

		cv::imshow("now",frame);
		cv::waitKey(30);
	}


	return 0;
}