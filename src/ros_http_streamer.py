#!/usr/bin/python
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import String

def namespace_callback(message):
    print('namespace:', message.data)

rospy.init_node("ros_http_streamer", anonymous=False)

rospy.Subscriber("rosx/namespace", String, namespace_callback)
