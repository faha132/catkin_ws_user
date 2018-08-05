
#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import tf 
import rospy

import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image


from math import sqrt
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry



from matplotlib import pyplot as plt

class PID:

    def __init__(self):
      self.pub_stop_start = rospy.Publisher(
          "FabianKhaled/stop_start",
          Int16,
          queue_size=100)
      self.pub_speed = rospy.Publisher("FabianKhaled/speed", Int16, queue_size=100)
      self.pub_steering = rospy.Publisher(
          "FabianKhaled/steering",
          UInt8,
          queue_size=100)
      self.sub_loc = rospy.Subscriber(
          "odom",Odometry, self.callback, queue_size=10)

  

    def callback(self,data):

      #print("WHEEEEEE")
      #make it move:    
      o = data.pose.pose.orientation
           
      #self.pub_speed.publish(0)
      #self.pub_stop_start.publish(150)
      #rospy.sleep(1)
      #print data
      
      print("#####")
      print("DEBUG")
      #print("data:",data)
            
      euler = tf.transformations.euler_from_quaternion([o.x, o.y, o.z, o.w])
      yaw = euler[2]
      
      print("newangle:", yaw)
      print("#####")
      #self.pub_steering.publish(newangle)    
      #self.pub_steering.publish(90)
#      self.pub_steering.publish(90)   #HARD LEFT
#      self.pub_steering.publish(0) #HARD RIGHT
      self.pub_stop_start.publish(1)
      #rospy.sleep(1)
      self.pub_speed.publish(110)
      #rospy.sleep(1)

  

def main(args):
  rospy.init_node('mini_PID', anonymous=True)
  mini_PID = PID()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
 


if __name__ == '__main__':
    main(sys.argv)

