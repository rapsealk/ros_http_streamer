#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageMessage():
    def __init__(self, message):
        self.header = message['header']
        self.height = message['height']
        self.width = message['width']
        self.encoding = "bgr8"
        self.is_bigendian = message['is_bigendian']
        self.step = message['step']
        self.data = message['data']

class ROSVision(object):

    def __init__(self):
        self.bridge = CvBridge()

    def callback(self, message):
        try:
            data = self.bridge.imgmsg_to_cv2(message, "bgr8")
            """
            cv2.imshow('roscv', data)
            cv2.waitKey(1)
            """
            ret, jpeg = cv2.imencode('.jpg', data)
            self.conn.send(jpeg.tostring())
        except CvBridgeError as e:
            sys.stderr.write("%s\n" % e)

    def run(self, conn):
        self.conn = conn
        namespace = conn.recv()
        rospy.Subscriber("/stream/{0}".format(namespace), Image, self.callback)
        rospy.spin()

if __name__ == '__main__':
    print('This cannot be loaded as main.')