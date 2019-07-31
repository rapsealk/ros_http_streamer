#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import sys
PYTHON27 = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if PYTHON27 in sys.path:
    sys.path.remove(PYTHON27)
import cv2

def main():
    rospy.init_node('camera_node', anonymous=False)
    image_pub = rospy.Publisher("rosx/stream/1", Image, queue_size=10)

    rate = rospy.Rate(30)   # Hz / fps

    import logging
    FORMAT = '%(asctime)-15s [%(levelname)s] %(message)s'
    logging.basicConfig(level=logging.DEBUG, format=FORMAT)

    camera = cv2.VideoCapture(0)
    bridge = CvBridge()

    if not camera.isOpened():
        camera.open(0)

    while camera.isOpened():
        ret, frame = camera.read()
        cv2.imshow("camera_node", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        try:
            message = bridge.cv2_to_imgmsg(frame)
            image_pub.publish(message)
        except CvBridgeError as e:
            print(e)

        #rospy.spinOnce()
        rate.sleep()

    camera.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
