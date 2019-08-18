/**
 * camera_image_node.cpp (c) 2019
 */
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <cv_bridge/cv_bridge.h>

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

    int seq = 0;

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

        sensor_msgs::Image image_message;
        std_msgs::Header header;
        header.seq = ++seq;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
        img_bridge.toImageMsg(image_message);
        image_pub.publish(image_message);

        ROS_INFO("Seq #%d Frame is displayed!", seq);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}