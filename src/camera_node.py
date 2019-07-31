#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

def main():
    rospy.init_node('camera_node', anonymous=False)
    image_pub = rospy.Publisher("rosx/stream/1", Image, queue_size=10)

    rate = rospy.Rate(30)   # Hz / fps

    camera = cv2.VideoCapture(0)
    bridge = CvBridge()

    while camera.isOpened:
        ret, frame = camera.read()
        cv2.imshow("camera_node", frame)

        try:
            message = bridge.cv2_to_imgmsg(frame)
            image_pub.publish(message)
        except CvBridgeError as e:
            print(e)

        rospy.spinOnce()
        rate.sleep()

if __name__ == "__main__":
    main()
