#!/usr/bin/python
# -*- coding: utf-8 -*-
from __future__ import print_function

import rospy
from sensor_msgs.msg import Image
import cv2

import cPickle
import zlib

import os
BASE_DIR = os.path.dirname(os.path.abspath(__file__)) 

import sys
import time
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

def parse_config():
    import json
    with open(BASE_DIR + '/config.json', 'r') as f:
        data = ''.join(f.readlines())
        data = json.loads(data)
    return data

def main():
    rospy.init_node("camera_bytes_node", anonymous=True)
    try:
        target_system_id = rospy.get_param('/ns02/mavros/target_system_id')
    except KeyError:
        target_system_id = 2
    publisher = rospy.Publisher("/stream/bytes/{0}".format(target_system_id), Image, queue_size=10)

    camera = cv2.VideoCapture(0)
    config = parse_config()
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, config['width'])
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, config['height'])
    #camera.set(cv2.CAP_PROP_FPS, 60)
    #camera.set(cv2.CAP_PROP_BUFFERSIZE, 3)
    #camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('X', '2', '6', '4'))

    rate = rospy.Rate(60)   # FPS
    
    if not camera.isOpened():
        camera.open(0)

    rospy.loginfo('camera.isOpened(): %r', camera.isOpened())

    while camera.isOpened() and not rospy.is_shutdown():
        ret, frame = camera.read()
        """
        cv2.imshow('frame', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        """
        height, width = frame.shape[:2]
        rospy.loginfo("height: %d, width: %d", height, width)
        channel = config['channel']
        rmatrix = cv2.getRotationMatrix2D((width/2, height/2), 180, 1)
        frame = cv2.warpAffine(frame, rmatrix, (width, height))
        rospy.loginfo("Affine Transformation :: Rotation 180'")
        frame = frame.flatten()
        rospy.loginfo("Transformation :: Flatten")
        framebytes = cPickle.dumps(frame)
        rospy.loginfo("cPickle.dumps(frame)")
        compressed = zlib.compress(framebytes, 5)
        rospy.loginfo("len(framebytes): %d, len(compressed): %d", len(framebytes), len(compressed))

        message = Image()
        message.height = height
        message.width = width
        message.encoding = "8UC3"
        message.is_bigendian = True
        message.data = compressed
        message.step = len(compressed) // height
        publisher.publish(message)
        rospy.loginfo("Message sent!\n---")

        rate.sleep()

    camera.release()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)
