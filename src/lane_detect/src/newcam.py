#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tim

area = 2150
kernel = np.ones((3,3),np.uint8)



if __name__ == '__main__':
# In ROS, nodes are uniquely named. If two nodes with the same
# node are launched, the previous one is kicked off. The
# anonymous=True flag means that rospy will choose a unique
# name for our 'listener' node so that multiple listeners can
# run simultaneously.
   rospy.init_node('listener', anonymous=True)

   rospy.Subscriber("chatter", String, callback)

   # spin() simply keeps python from exiting until this node is stopped
   