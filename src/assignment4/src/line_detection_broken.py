#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
import numpy as np
# from std_msgs.msg import String
# from geometry_msgs.msg import Pose
# from geometry_msgs.msg import Point
# from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sklearn import linear_model 
#import matplotlib
#matplotlib.use('Agg')
# from matplotlib import pyplot as plt

# from __future__ import print_function

class image_converter:

  def __init__(self):
      
    self.image_pub_yuv = rospy.Publisher("/image_processing/lines_from_yuv",Image, queue_size=1)
    self.image_pub_bgr = rospy.Publisher("/image_processing/lines_from_bgr",Image, queue_size=1)
    self.image_pub_hsv = rospy.Publisher("/image_processing/lines_from_hsv",Image, queue_size=1)
    self.image_pub_lined = rospy.Publisher("/image_processing/lined",Image, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    #hsv colorspace

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_bound = np.array([0,0,200])
    upper_bound = np.array([255,255,255])

    #Threshold the HSV image to get only white areas
    maskhsv = cv2.inRange(hsv, lower_bound, upper_bound)

    # Bitwise-AND mask and original image
    reshsv = cv2.bitwise_and(hsv,hsv, mask=maskhsv)

    #luminance layer of YUV (Greyscale)
    gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    bi_gray_max = 255
    bi_gray_min = 245
    _,thresh1=cv2.threshold(gray, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    #bgr
    light = np.array([255,255,255])
    dark = np.array([200,200,200])

    maskbgr = cv2.inRange(cv_image, dark, light)
    #not a big difference
    resbgr = cv2.bitwise_and(cv_image,cv_image, mask=maskbgr)
    try:
      #no good encoding found bgr atleast delivers visible output
      self.image_pub_hsv.publish(self.bridge.cv2_to_imgmsg(reshsv, encoding="bgr8"))
      self.image_pub_yuv.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8" ))
      self.image_pub_bgr.publish(self.bridge.cv2_to_imgmsg(resbgr, encoding="bgr8"))
    except CvBridgeError as e:
      print(e)

    #lane detection 

    #split image in halves
    left = thresh1[:,:int(thresh1.shape[1]/2)]
    right = thresh1[:,int(thresh1.shape[1]/2):]

    dots1 = np.where(right == 255)
    points1 = np.zeros((dots1[0].shape[0],1,2), np.int8 )
    for i,x in enumerate(dots1[0]):
      points1[i][0][0]=x
    for i,y in enumerate(dots1[1]):
      points1[i][0][1]=y

    
    lane_model1 = linear_model.RANSACRegressor()
    lane_model1.fit(points1[:,:,0], points1[:,:,1])

    b2 = lane_model1.predict(0).item(0)
    print b2
    m2 = (lane_model1.predict(100).item(0) - b2)/100
    print m2

    # draw the lines

    height, width = cv_image.shape[:2]
    cv2.line(cv_image, (int(width/2),int(b2)) ,(width,int(b2+width*m2)), 
	     (0,0,255), thickness=2, lineType=cv2.LINE_AA)
    
#    dots1 = np.where(left == 255)
#    points1 = np.zeros((dots1[0].shape[0],1,2), np.int8 )
#    for i,x in enumerate(dots1[0]):
#      points1[i][0][0]=x
#    for i,y in enumerate(dots1[1]):
#      points1[i][0][1]=y
#
#    
#    lane_model1 = linear_model.RANSACRegressor()
#    lane_model1.fit(points1[:,:,0], points1[:,:,1])
#
#    b2 = lane_model1.predict(0).item(0)
#    m2 = (lane_model1.predict(100).item(0) - b2)/100
#
#    # draw the lines
#    x_max = int(cv_image.shape[1])    
#    cv2.line(cv_image[120:,:], (0,int(b2)), (x_max,int(b2+(x_max/2)*m2)), 
#	     (0,0,255), thickness=2, lineType=cv2.LINE_AA)
#
#
    # publish an image to the control topic
    try:
      self.image_pub_lined.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
