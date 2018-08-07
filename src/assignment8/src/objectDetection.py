#!/usr/bin/env python
import numpy as np
import sys
import rospy
from std_msgs.msg import Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class obst_detector:

  def __init__(self):
    self.obstacle_pub = rospy.Publisher("/obstacle/warning",Int32, queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/fabiankhaled/app/camera/depth/image_raw", Image,self.obstacle_callback, queue_size=1)

  def obstacle_callback(self,data):
    try:
      depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
    except CvBridgeError as e:
      print(e)
    depth_array = np.array(depth_image, dtype=np.uint16)
    height, width = depth_array.shape[0:2]
    
    curr_lane = 1
    switch = 5000
    
    #only corners and centers
    for y in xrange(0,height,height//3-1):
      for x in xrange(0,width,width//3-1):
        if depth_array[y,x] >= 60:
    
    #mounted objects have closer range 
          switch = min(depth_array[y,x],switch)
    try:
        #0 would be inside the camera -> flag for nothing detected
        self.obstacle_pub.publish(Int32(switch))
    except CvBridgeError as e:
        print(e)


def main(args):
  rospy.init_node('obst_detector', anonymous=True)
  ic = obst_detector()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
