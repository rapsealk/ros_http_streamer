#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

import rospy
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension

def callback(message):
    rospy.loginfo("ByteMultiArray")
    rospy.loginfo("data: %s", str(message.data))
    rospy.loginfo("layout.dim[0].size: %d", message.layout.dim[0].size)
    rospy.loginfo("---")

def main():
    rospy.init_node("bytes_subscriber", anonymous=True)
    rospy.Subscriber("/bytes", ByteMultiArray, callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)
