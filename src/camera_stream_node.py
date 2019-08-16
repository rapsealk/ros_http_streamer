#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

try:
    import cPickle as pickle
except:
    import pickle
import zlib

import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__)) 

import sys
import time
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

publisher_01 = None
publisher_02 = None
publisher_03 = None

bridge = CvBridge()

import json
with open(BASE_DIR + '/config.json', 'r') as f:
    data = ''.join(f.readlines())
    data = json.loads(data)

WIDTH = data['width']
HEIGHT = data['height']
CHANNEL = data['channel']

# TODO: lambda
def callback_01(message):
    global publisher_01
    log = """Image /stream/bytes/1
    - height: %d
    - width: %d
    - step: %d
    - encoding: %s
    - is_bigendian: %r
    ---""" % (message.height, message.width, message.step, message.encoding, message.is_bigendian)
    rospy.loginfo(log)

    decompressed = zlib.decompress(message.data)
    flatten = pickle.loads(decompressed)
    frame = np.reshape(flatten, (HEIGHT, WIDTH, CHANNEL))
    try:
        image = bridge.cv2_to_imgmsg(frame, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("%s" % e)
        return
    publisher_01.publish(image)
    rospy.loginfo("Image01 published!")

def callback_02(message):
    global publisher_02
    log = """Image /stream/bytes/2
    - height: %d
    - width: %d
    - step: %d
    - encoding: %s
    - is_bigendian: %r
    ---""" % (message.height, message.width, message.step, message.encoding, message.is_bigendian)
    rospy.loginfo(log)

    decompressed = zlib.decompress(message.data)
    flatten = pickle.loads(decompressed)
    frame = np.reshape(flatten, (HEIGHT, WIDTH, CHANNEL))
    try:
        image = bridge.cv2_to_imgmsg(frame, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("%s" % e)
        return
    publisher_02.publish(image)
    rospy.loginfo("Image02 published!")

def callback_03(message):
    global publisher_03
    log = """Image /stream/bytes/3
    - height: %d
    - width: %d
    - step: %d
    - encoding: %s
    - is_bigendian: %r
    ---""" % (message.height, message.width, message.step, message.encoding, message.is_bigendian)
    rospy.loginfo(log)

    decompressed = zlib.decompress(message.data)
    flatten = pickle.loads(decompressed)
    frame = np.reshape(flatten, (HEIGHT, WIDTH, CHANNEL))
    try:
        image = bridge.cv2_to_imgmsg(frame, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("%s" % e)
        return
    publisher_03.publish(image)
    rospy.loginfo("Image03 published!")

def main():
    global publisher_01, publisher_02, publisher_03
    rospy.init_node("camera_stream_node", anonymous=False)
    publisher_01 = rospy.Publisher("/stream/1", Image, queue_size=10)
    publisher_02 = rospy.Publisher("/stream/2", Image, queue_size=10)
    publisher_03 = rospy.Publisher("/stream/3", Image, queue_size=10)
    rospy.Subscriber("/stream/bytes/1", Image, callback_01)
    rospy.Subscriber("/stream/bytes/2", Image, callback_02)
    rospy.Subscriber("/stream/bytes/3", Image, callback_03)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)
