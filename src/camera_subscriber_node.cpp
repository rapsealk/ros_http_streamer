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

void callback(const sensor_msgs::Image::ConstPtr& message) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("Subscriber", cv_ptr->image);
    cv::waitKey(3);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "camera_subscriber_node");
    ros::NodeHandle nh;

    ros::Subscriber image_sub = nh.subscribe("/camera/image/2", 100, callback);

    ros::spin();

    return 0;
}