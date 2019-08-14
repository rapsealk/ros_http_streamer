#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

import sys
import time
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

import rospy
import cv2
import numpy as np
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import pickle
import zlib

publisher_01 = None
publisher_02 = None
publisher_03 = None

bridge = CvBridge()

# TODO: lambda
def callback_01(message):
    global publisher_01
    log = """ByteMultiArray /stream/bytes/1
    - data: %s
    - layout
      - dim[0].label: %s
      - dim[0].size: %d
    ---""" % (str(message.data), message.layout[0].label, message.layout.size)
    rospy.loginfo(log)
    raw_data = bytes(bytearray(message.data))
    height, width, channel = message.layout[0].size, message.layout[1].size, message.layout[2].size
    decompressed = zlib.decompress(raw_data)
    matrix = pickle.loads(decompressed)
    frame = np.reshape(matrix, (height, width, channel))
    try:
        image = bridge.cv2_to_imgmsg(frame, "bgr8")
        publisher_01.publish(image)
        sys.stdout.write("[%s] Frame published!\n" % time.time())
    except CvBridgeError as e:
        sys.stderr.write("%s\n" % e)

def callback_02(message):
    global publisher_02
    log = """ByteMultiArray /stream/bytes/2
    - data: %s
    - layout
      - dim[0].label: %s
      - dim[0].size: %d
    ---""" % (str(message.data), message.layout[0].label, message.layout.size)
    rospy.loginfo(log)
    raw_data = bytes(bytearray(message.data))
    height, width, channel = message.layout[0].size, message.layout[1].size, message.layout[2].size
    decompressed = zlib.decompress(raw_data)
    matrix = pickle.loads(decompressed)
    frame = np.reshape(matrix, (height, width, channel))
    try:
        image = bridge.cv2_to_imgmsg(frame, "bgr8")
        publisher_02.publish(image)
        sys.stdout.write("[%s] Frame published!\n" % time.time())
    except CvBridgeError as e:
        sys.stderr.write("%s\n" % e)

def callback_03(message):
    global publisher_03
    log = """ByteMultiArray /stream/bytes/3
    - data: %s
    - layout
      - dim[0].label: %s
      - dim[0].size: %d
    ---""" % (str(message.data), message.layout[0].label, message.layout.size)
    rospy.loginfo(log)
    raw_data = bytes(bytearray(message.data))
    height, width, channel = message.layout[0].size, message.layout[1].size, message.layout[2].size
    decompressed = zlib.decompress(raw_data)
    matrix = pickle.loads(decompressed)
    frame = np.reshape(matrix, (height, width, channel))
    try:
        image = bridge.cv2_to_imgmsg(frame, "bgr8")
        publisher_03.publish(image)
        sys.stdout.write("[%s] Frame published!\n" % time.time())
    except CvBridgeError as e:
        sys.stderr.write("%s\n" % e)

def main():
    global publisher_01, publisher_02, publisher_03
    rospy.init_node("camera_stream_node", anonymous=False)
    publisher_01 = rospy.Publisher("/stream/1", Image, queue_size=10)
    publisher_02 = rospy.Publisher("/stream/2", Image, queue_size=10)
    publisher_03 = rospy.Publisher("/stream/3", Image, queue_size=10)
    rospy.Subscriber("/stream/bytes/1", ByteMultiArray, callback_01)
    rospy.Subscriber("/stream/bytes/2", ByteMultiArray, callback_02)
    rospy.Subscriber("/stream/bytes/3", ByteMultiArray, callback_03)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)
