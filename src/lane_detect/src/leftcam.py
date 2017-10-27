#!/usr/bin/env python
from __future__ import print_function
import roslib
#roslib.load_manifest('package.xml')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np



def nothing(x):
    pass

area = 1500
#cv2.namedWindow('areathl')
#cv2.createTrackbar('areath', 'areathl', 2000, 3000, nothing)

kernel = np.ones((3,3),np.uint8)

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/camera/image_raw1",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("camera1/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    im1 = cv_image
    #cv2.imshow('areathl',im1)
    blank = np.zeros((im1.shape[0],im1.shape[1],3), np.uint8)
    rows,cols = blank.shape[:2]
    img = cv2.cvtColor(im1, cv2.COLOR_BGR2GRAY)
    thgauss = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,\
         cv2.THRESH_BINARY,(2*60 + 3),80-100)
    #cv2.imshow('gaussorg',thgauss)

    thgauss2,contours,hierarchy = cv2.findContours(thgauss,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #area = cv2.getTrackbarPos('areath', 'areathl')

    for n, contours in enumerate(contours):
        if cv2.contourArea(contours) > area :
            x,y,w,h = cv2.boundingRect(contours)
            if True :
                #cv2.rectangle(im1,(x,y),(x+w,y+h),(0,255,0),2)
                cv2.drawContours(im1,[contours],-1,(0,255,0),2)
                cv2.drawContours(blank,[contours],-1,(255,255,255),-1)
                extLeft = tuple(contours[contours[:, :, 0].argmin()][0])
                extRight = tuple(contours[contours[:, :, 0].argmax()][0])
                [vx,vy,x,y] = cv2.fitLine(contours, cv2.DIST_L2,0,0.01,0.01)
                lefty = int(((extLeft[0]-x)*vy/vx) + y)
                righty = int(((extRight[0]-x)*vy/vx)+y)
                #cv2.line(blank,(extRight[0]-1,righty),(extLeft[0],lefty),(255,255,255),10)
                
                
    final = cv2.cvtColor(blank, cv2.COLOR_BGR2GRAY)
    #cv2.imshow('im1',im1)
    cv2.imshow('final',final)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(final, "mono8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  ic = image_converter()
  rospy.init_node('image_processor', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)



