#!/usr/bin/env python
# roslib.load_manifest('my_package')
#--------------------------
import roslib
import sys
import rospy
import cv2
from sensor_msgs.msg import Image

class image_converter:

  def __init__(self):
    self.image_pub_hsv = rospy.Publisher("/assignment4kf/hsv",Image, queue_size=1)

    self.image_pub_bw = rospy.Publisher("/assignment34321/bin_img/bw",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)


    #make it hsv
    hsv=cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    #bi_gray
    bi_gray_max = 255
    bi_gray_min = 240 #210
    ret,thresh1=cv2.threshold(hsv, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY);

    try:
      self.image_pub_hsv.publish(self.bridge.cv2_to_imgmsg(hsv, "mono8"))
      self.image_pub_bw.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
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