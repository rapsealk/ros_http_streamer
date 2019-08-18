/**
 * camera_image_node.cpp (c) 2019
 */
#include <vector>

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>

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

    cap.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);

    if (!cap.isOpened()) {
        ROS_ERROR("Camera is not opened!");
        return -1;
    }

    int seq = 0;

    while (ros::ok() && cap.isOpened()) {
        cap.read(image);
        if (image.empty()) {
            ROS_ERROR("Image is empty!");
            break;
        }

        cv::imshow("Frame", image);
        ROS_INFO("rows: %d, cols: %d, dims: %d", image.rows, image.cols, image.dims);
        ROS_INFO("Image size: %d byte(s)", sizeof(uchar) * image.rows * image.cols * image.dims);

        ros::Time time = ros::Time::now();
        /* Compression */
        //ROS_INFO("Compression: %fs, size: %d", ros::Time::now() - time, size);

        if (cv::waitKey(25) == 'q') {
            break;
        }

        cv::Mat flatten = image.reshape(1, image.cols * image.rows);

        sensor_msgs::Image image_message;
        std_msgs::Header header;
        header.seq = ++seq;
        header.stamp = ros::Time::now();
        cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image);
        img_bridge.toImageMsg(image_message);
        image_pub.publish(image_message);

        ROS_INFO("Seq #%d Frame is displayed!\n---", seq);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}