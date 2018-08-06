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

       
    _, contours, _ = cv2.findContours(thresh1, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    lane_contours = [None, None]
    first_size = 0
    secnd_size = 0

for contour in contours:
    test = cv2.contourArea(contour)
        if test > first_size:
            lane_contours[1] = lane_contours[0]
            secnd_size = first_size
            lane_contours[0] = contour
            first_size = test
        elif test > secnd_size:
            lane_contours[1] = contour
            secnd_size = test
        
    lane_models = [None, None]
    print(lane_contours[0].shape)
    print type(lane_contours[0])
    for i in range(len(lane_contours)):
        points = lane_contours[i]
        lane_models[i] = linear_model.RANSACRegressor()
        lane_models[i].fit(points[:,:,0], points[:,:,1])

    b1 = lane_models[0].predict(0).item(0)
    b2 = lane_models[1].predict(0).item(0)
    m1 = (lane_models[0].predict(100).item(0) - b1)/100
    m2 = (lane_models[1].predict(100).item(0) - b2)/100

    print("m1: " + str(m1) + " b1: " + str(b1) )
    print("m2: " + str(m2) + " b2: " + str(b2) )


    height, width = cv_image[:2]
    width = int(cv_image.shape[1])    
    cv2.line(cv_image, (0,int(b1)), (width,int(b1+width*m1)), 
              (0,0,255), thickness=2, lineType=cv2.LINE_AA)
    cv2.line(cv_image, (0,int(b2)), (width,int(b2+width*m2)), 
              (0,0,255), thickness=2, lineType=cv2.LINE_AA)

    try:
        self.image_pub_lined.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
        print(e)

def main(args):
  rospy.init_node('simple_node', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
