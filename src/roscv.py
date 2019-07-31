#!/usr/bin/python3
# -*- coding: utf-8 -*-
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

    def callback(self, data):
        data_message = ImageMessage(data)
        try:
            data_message = CvBridge().imgmsg_to_cv2(data_message)
            ret, jpeg = cv2.imencode('.jpg', data_message)
            self.conn.send(jpeg.tostring())
        except CvBridgeError as e:
            print(e)

    def run(self, conn):
        self.conn = conn
        namespace = conn.recv()
        rospy.Subscriber("rosx/stream/{0}".format(namespace), Image, self.callback)
        rospy.spin()

if __name__ == '__main__':
    print('This cannot be loaded as main.')