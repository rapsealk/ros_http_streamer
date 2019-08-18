/**
 *
 */
#include <opencv2/opencv.hpp>
#include <iostream>

#define WIDTH 640
#define HEIGHT 480

int main()
{
    cv::VideoCapture cap(0);
    cv::Mat image;

    if (!cap.isOpened()) {
        std::cerr << "Camera is not opened!" << std::endl;
        return -1;
    }

    while (true) {
        cap.read(image);
        if (image.empty()) {
            std::cerr << "Image is empty!" << std::endl;
            break;
        }

        cv::imshow("Frame", image);

        if (cv::waitKey(25) == 'q') {
            break;
        }
    }

    return 0;
}