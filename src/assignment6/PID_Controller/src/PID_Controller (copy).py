#!/usr/bin/env python

import math
import rospy
import sys
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16 

class PID_Controller:
	def __init__(self):
		self.last_time = rospy.Time.now()
		self.yaw= 0. 
		self.Stop= 1
		self.setpoint= math.pi
		
		self.aq_error= 0.
		self.last_error= 0.

		dummya_1 = rospy.Subscriber("/localization/odom/0", Odometry, self.call_back)	#subscribe to topic
		dummya_2 = rospy.Subscriber("/PID_Stop_Start/", Int16, self.call_back_Stop)
		
		self.vel_pub = rospy.Publisher("/manual_control/speed/",Int16, queue_size=1)
		self.str_pub = rospy.Publisher("/steering/",Int16, queue_size=1)
		
		
		
		

	def call_back_Stop (self,raw_msgs):
		self.Stop= raw_msgs.data
		vel = Int16()
		
		if self.Stop == 0:
			print("Stop")
			vel.data = 0.
		else:
			vel.data = 200
			print("Start")

		self.vel_pub.publish(vel)

	def call_back (self,raw_msgs):
		
		#rospy.loginfo(raw_msgs)
		orientation= raw_msgs.pose.pose.orientation
		#rospy.loginfo(orientation)
		orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
		orientation_in_Red= tf.transformations.euler_from_quaternion(orientation_array) #change the read-out data from Euler to Rad
		self.yaw =orientation_in_Red[2]
		self.PID()
		
	
	def PID(self):	
		Kp= 2
		Ki= 0.9
		Kd= 0.3
		
		error= self.setpoint - self.yaw
		self.aq_error = self.aq_error + error
		if self.aq_error > 1000:
			self.aq_error=1000
		elif self.aq_error < -1000:
			self.aq_error = -1000

		current_time = rospy.Time.now()
		dif_time = (current_time - self.last_time).to_sec()

		print(current_time)
		print(self.last_time)
		print(dif_time)

		
		PID= Kp * error + Ki * self.aq_error * dif_time + Kd * (error - self.last_error) / dif_time
		self.last_time = current_time
		self.last_error = error

		self.str_pub.publish(PID)


def main(args):

	rospy.init_node("Local_GPS_data") # have to define node for ROS
	
	control = PID_Controller() # call the class
	try:
		rospy.spin()
	except:
		print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)

