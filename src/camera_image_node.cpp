/**
 * camera_image_node.cpp (c) 2019
 */
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv2/opencv.hpp>
#include <iostream>

#define WIDTH 640
#define HEIGHT 480

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_image_node");
    ros::NodeHandle nh;

    ros::Publisher image_pub = nh.advertise<sensor_msgs::Image>("/camera/image/2", 100);

    ros::Rate rate(10);

    cv::VideoCapture cap(0);
    cv::Mat image;

    if (!cap.isOpened()) {
        std::cerr << "Camera is not opened!" << std::endl;
        return -1;
    }

    while (ros::ok() && cap.isOpened()) {
        cap.read(image);
        if (image.empty()) {
            std::cerr << "Image is empty!" << std::endl;
            break;
        }

        cv::imshow("Frame", image);

        if (cv::waitKey(25) == 'q') {
            break;
        }

        ROS_INFO("Frame is displayed!");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}