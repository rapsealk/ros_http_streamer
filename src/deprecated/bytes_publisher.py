#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import time
import logging
FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
logging.basicConfig(level=logging.DEBUG, format=FORMAT)

import rospy
from std_msgs.msg import ByteMultiArray, MultiArrayLayout, MultiArrayDimension

def main():
    rospy.init_node("bytes_publisher", anonymous=True)
    publisher = rospy.Publisher("/bytes", ByteMultiArray, queue_size=10)
    while not rospy.is_shutdown():
        number = int(raw_input("Number: "))
        if number == 0:
            break
        text = bytearray(number)
        """
        text = raw_input("Input: ")
        if text == 'q':
            break
        rospy.loginfo('Text: %s, type: %s', text, type(text))
        text = text.encode("utf-8")
        rospy.loginfo('Encoded: %s, type: %s', text, type(text))
        rospy.loginfo('ByteArray: %s', bytearray(text))
        """
        
        dimension = MultiArrayDimension()
        dimension.label = "size"
        dimension.size = len(text)
        dimension.stride = len(text)
        layout = MultiArrayLayout()
        layout.dim = [dimension]
        layout.data_offset = 0
        message = ByteMultiArray()
        message.layout = layout
        message.data = bytearray(text)
        publisher.publish(message)
    #rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException as e:
        sys.stderr.write("%s\n" % e)
