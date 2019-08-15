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
from sensor_msgs.msg import Image

import pickle
import zlib

def main():
    rospy.init_node("test_camera_bytes_node", anonymous=True)
    try:
        target_system_id = rospy.get_param('/ns01/mavros/target_system_id')
    except KeyError:
        target_system_id = 1
    publisher = rospy.Publisher("/test/stream/bytes/{0}".format(target_system_id), Image, queue_size=10)

    camera = cv2.VideoCapture(0)
    rate = rospy.Rate(30)   # FPS
    
    if not camera.isOpened():
        camera.open(0)

    print('camera.isOpened():', camera.isOpened())

    while camera.isOpened() and not rospy.is_shutdown():
        ret, frame = camera.read()

        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        height, width = frame.shape[:2]
        rospy.loginfo("height: %d, width: %d", height, width)
        channel = 3
        rmatrix = cv2.getRotationMatrix2D((width/2, height/2), 180, 1)
        frame = cv2.warpAffine(frame, rmatrix, (width, height))

        frame = frame.flatten()
        framebytes = pickle.dumps(frame)
        compressed = zlib.compress(framebytes, 5)
        rospy.loginfo("len(framebytes): %d, len(compressed): %d, len(bytearray(compressed)): %d", len(framebytes), len(compressed), len(bytearray(compressed)))
        rospy.loginfo("type(compressed): %s, type(bytearray(compressed)): %s", type(compressed), type(bytearray(compressed)))
        """
        dimension = MultiArrayDimension()
        dimension.label = "size"
        dimension.size = len(compressed)
        dimension.stride = len(compressed)
        layout = MultiArrayLayout()
        layout.dim = [dimension]
        layout.data_offset = 0
        message = ByteMultiArray()
        message.layout = layout
        message.data = compressed#list(bytearray(compressed))
        """
        image = Image()
        image.height = height
        image.width = width
        image.encoding = "8UC3"
        image.is_bigendian = True
        image.data = compressed
        image.step = len(compressed) // height

        publisher.publish(image)

        rate.sleep()

    camera.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)