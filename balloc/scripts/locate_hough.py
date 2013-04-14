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
    self.bridge = CvBridge()
    #self.image = cv.CreateMat(1280,720,16)
    self.mute = False		
    self.image_sub = rospy.Subscriber("/usb_cam/image_raw",Image,self.im_callback)
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
      "Waiting"
      try:
        image = np.asarray(cv2.cv.CloneMat(self.image))
      except AttributeError:
        rospy.sleep(1)
        continue 
      image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
      image = cv2.GaussianBlur(image, (3,3), 2)
      edges = cv2.Canny(image, 50, 20)
      #c = edges
      #c = cv2.HoughCircles(edges, cv2. , 2, 100)
      circles =  cv2.HoughCircles(image, cv2.cv.CV_HOUGH_GRADIENT, 2, 20, maxRadius=100)
      print circles
      if circles is not None:
        for c in circles[0]:
          cv2.circle(image, (c[0],c[1]), c[2], (0,255,0),2)
      #convert to HSV space
      #image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
      #Filter for yellow balls
      #chan = cv.CloneMat(image)
      #chan = cv2.split(np.asarray(image))
      #cv.Threshold(image,image,228,255,cv.CV_THRESH_BINARY) 		
      #cv.Dilate(image,image,None,2)
      #cv.Erode(image,image,None,2)
      cv2.imshow("Image window", image)
      cv2.imshow("Edges", edges)
      if cv2.waitKey(2) == 1048603:
          rospy.signal_shutdown("You asked me to you bastard")
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

