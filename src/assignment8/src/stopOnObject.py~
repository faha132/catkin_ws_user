#!/usr/bin/env python
import numpy as np
import sys
import rospy
from std_msgs.msg import Bool, Int32, UInt8
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from assignment7 import field_Controller
from objectDetection import obst_detector

class obst_driver:

   def __init__(self):
       #Tunable
        self.gain_dist = 90
        self.speed_val = 300
        #notTunable
        self.vel_pub = rospy.Publisher("/fabiankhaled/manual_control/speed",
                                       Int16, queue_size=100)
        self.str_pub = rospy.Publisher("/fabiankhaled/steering",UInt8, queue_size=1)
        #detection in different node launchfile assignment8/launch
        self.obstacle_sub = rospy.Subscriber("/obstacle/warning",Int32, self.callback,
                                             squeue_size=1)
        self.nav_stop_pub = rospy.Publisher("/nav/stop", Bool, queue_size = 1)
        self.nav_lane_change = rospy.Publisher("/Lane_change", Bool, queue_size = 1)
        self.noticed = False
        self.last_dist = 0

   def callback(self,data):
        if data.data != 0:
            if self.noticed == False:
                self.nav_stop_pub.publish(Bool(True))
                self.vel_pub.publish(Int16(0))
                #set steering move away from object in a straight line 
                #test
                self.str_pub.publish(UInt8(100))
                self.nav_lane_change.publish(Bool(True))
                self.last_dist = data.data
                self.noticed = True
        if noticed:
            #make some space for lane switch manuever
            if self.last_dist >= data.data-self.gain_dist:
                self.vel_pub.publish(Int16((-1)*self.speed_val))
            elif data.data == 0:
                noticed = False
                self.nav_stop_pub.publish(Bool(False))
            else:
                self.nav_stop_pub.publish(Bool(False))

def main(args):
    rospy.init_node('obst_driver', anonymous=True)
    driv = obst_driver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
