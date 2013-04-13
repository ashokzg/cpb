#!/usr/bin/env python
import roslib
roslib.load_manifest('balloc')
import sys
import rospy
import cv
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from copy import deepcopy
import numpy as np

class ball_locator:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)
    cv.NamedWindow("Pool table tracker", 1)
    self.bridge = CvBridge()
    #self.image = cv.CreateMat(1280,720,16)
    self.mute = False		
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.im_callback)
    rospy.sleep(0.333)
    self.locator()

      		 
  def im_callback(self,data):
    try:
      if self.mute == False:	
      	self.image = self.bridge.imgmsg_to_cv(data, "bgr8")
      	
    except CvBridgeError, e:
      print e

    try:
      self.image_pub.publish(self.bridge.cv_to_imgmsg(self.image, "bgr8"))
    except CvBridgeError, e:
      print e

  def locator(self):
    while not rospy.is_shutdown():
      image = cv.CloneMat(self.image) 
      cv.Smooth(image,image, cv.CV_GAUSSIAN, 15, 15)
      #convert to HSV space
      image = cv.fromarray(cv2.cvtColor(np.asarray(image),cv2.cv.CV_BGR2HSV))      
      #Filter for yellow balls
      #chan = cv.CloneMat(image)
      #chan = cv2.split(np.asarray(image))
      #cv.Threshold(image,image,228,255,cv.CV_THRESH_BINARY) 		
      #cv.Dilate(image,image,None,2)
      #cv.Erode(image,image,None,2)
      cv.ShowImage("Image window", image)
      cv.WaitKey(3)
      rospy.sleep(0.05)# sleeping for 50 ms	

def main(args):
  rospy.init_node('ball_loc', anonymous=True)
  bl = ball_locator()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

