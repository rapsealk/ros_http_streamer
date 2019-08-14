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
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension

import pickle
import zlib

def main():
    rospy.init_node("camera_bytes_node", anonymous=True)
    try:
        target_system_id = rospy.get_param('/ns02/mavros/target_system_id')
    except KeyError:
        target_system_id = 2
    publisher = rospy.Publisher("/stream/bytes/{0}".format(target_system_id), ByteMultiArray, queue_size=10)

    camera = cv2.VideoCapture(0)
    rate = rospy.Rate(30)   # FPS
    
    if not camera.isOpened():
        camera.open(0)

    print('camera.isOpened():', camera.isOpened())

    while camera.isOpened() and not rospy.is_shutdown():
        ret, frame = camera.read()
        height, width = frame.shape[:2]
        rospy.loginfo("height: %d, width: %d", height, width)
        channel = 3
        rmatrix = cv2.getRotationMatrix2D((width/2, height/2), 180, 1)
        frame = cv2.warpAffine(frame, rmatrix, (width, height))
        frame = frame.flatten()
        framebytes = pickle.dumps(frame)
        compressed = zlib.compress(framebytes, 5)
        rospy.loginfo("len(framebytes): %d, len(compressed): %d", len(framebytes), len(compressed))
        """
        hdimension = MultiArrayDimension()
        hdimension.label = "height"
        hdimension.size = height
        hdimension.stride = channel * width * height

        wdimension = MultiArrayDimension()
        wdimension.label = "width"
        wdimension.size = width
        wdimension.stride = channel * width

        cdimension = MultiArrayDimension()
        cdimension.label = "channel"
        cdimension.size = channel
        cdimension.stride = channel
        """
        dimension = MultiArrayDimension()
        dimension.label = "bytes"
        dimension.size = len(compressed)
        dimension.stride = len(compressed)

        layout = MultiArrayLayout()
        layout.dim = [dimension]#[hdimension, wdimension, cdimension]
        layout.data_offset = 0
        message = ByteMultiArray()
        message.layout = layout
        message.data = tuple([i for i in bytearray(compressed)])#bytearray(compressed)
        publisher.publish(message)

        rate.sleep()

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)
