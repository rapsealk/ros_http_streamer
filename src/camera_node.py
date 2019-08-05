#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def main():
    rospy.init_node("camera_node", anonymous=False)
    target_system_id = rospy.get_param('/ns01/mavros/target_system_id')
    publisher = rospy.Publisher("/stream/{0}".format(target_system_id), Image, queue_size=10)
    bridge = CvBridge()

    camera = cv2.VideoCapture(0)
    rate = rospy.Rate(30)   # FPS
    
    if not camera.isOpened():
        camera.open(0)

    print('camera.isOpened():', camera.isOpened())

    while camera.isOpened() and not rospy.is_shutdown():
        ret, frame = camera.read()
        height, width = frame.shape[:2]
        rmatrix = cv2.getRotationMatrix2D((width/2, height/2), 180, 1)
        frame = cv2.warpAffine(frame, rmatrix, (width, height))
        #cv2.imshow('camera_node', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        try:
            message = bridge.cv2_to_imgmsg(frame, "bgr8")
            publisher.publish(message)
        except CvBridgeError as e:
            sys.stderr.write("%s\n" % e)

        rate.sleep()

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)
