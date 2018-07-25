#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float32

def keks(yawmsg):
    msg = "i heard: " + str(yawmsg.data)
    rospy.loginfo(msg)
    pub.publish(msg)
    print(yawmsg.data) 
    

rospy.init_node("simple_node332")

pub = rospy.Publisher("/assignment1_publisher_subscriber", String , queue_size=10)
r = rospy.Rate(10) # 10hz

rospy.Subscriber("/yaw", Float32 , keks)

rospy.spin()
