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
from sensor_msgs.msg import Image

import pickle
import zlib

def callback(message):
    rospy.loginfo("Image")
    decompressed = zlib.decompress(message.data)
    flatten = pickle.loads(decompressed)
    frame = np.reshape(flatten, (480, 640, 3))
    cv2.imshow('image', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        sys.exit(0)
    rospy.loginfo("---")

def main():
    rospy.init_node("test_bytes_image_node", anonymous=True)
    try:
        target_system_id = rospy.get_param('/ns01/mavros/target_system_id')
    except KeyError:
        target_system_id = 1
    rospy.Subscriber("/test/stream/bytes/{0}".format(target_system_id), Image, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)