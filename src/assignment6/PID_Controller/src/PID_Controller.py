#!/usr/bin/env python

import math
import rospy
import sys
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16, UInt8, Float64

class PID_Controller:
	def __init__(self):
		self.last_time = rospy.Time.now()
		self.yaw= 0. 
		self.Stop= 0
		self.setpoint= math.pi
		
		self.aq_error= 0.
		self.last_error= 0.

		self.dummya_1 = rospy.Subscriber("/localization/odom/0", Odometry, self.call_back)	#subscribe to topic
		self.dummya_2 = rospy.Subscriber("/PID_Stop_Start/", Int16, self.call_back_Stop)
		
		self.vel_pub = rospy.Publisher("/fabiankhaled/manual_control/speed",Int16, queue_size=1)
		self.str_pub = rospy.Publisher("/fabiankhaled/steering",UInt8, queue_size=1)
		self.m_yaw = rospy.Publisher("/measured_yaw",Float64, queue_size=1)
		self.set_point = rospy.Publisher("/set_point",Float64, queue_size=1)
		self.e=rospy.Publisher("/Error", Float64, queue_size=1)


	def call_back_Stop (self,raw_msgs):
		self.Stop= raw_msgs.data
		vel = Int16()
		#print("Stop: {0}".format(self.Stop))
		
		if self.Stop == 0:
			print("Stop")
			vel.data = 0
		else:
			vel.data = 1000
			print("Start")

		self.vel_pub.publish(vel)

	def call_back (self,raw_msgs):
		
		#rospy.loginfo(raw_msgs)
		orientation= raw_msgs.pose.pose.orientation
		#rospy.loginfo(orientation)
		orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
		orientation_in_Red= tf.transformations.euler_from_quaternion(orientation_array) #change the read-out data from Euler to Rad
		self.yaw =orientation_in_Red[2]
		# print(type(self.yaw))
		
		meas_1 = Float64(self.yaw)
		meas_2 = Float64(self.setpoint)
		#meas_3 = Float32(error)
		self.set_point.publish(0)
		self.m_yaw.publish(meas_1)
		#print(meas_1)
		
		self.PID()
		
		
	
	def PID(self):	
		Kp= 180
		Ki= 250
		Kd= 200
		
		error= (self.setpoint - self.yaw) - math.pi
		self.aq_error = self.aq_error + error
		if self.aq_error > 10:
			self.aq_error=10
		elif self.aq_error < -10:
			self.aq_error = -10

		current_time = rospy.Time.now()
		dif_time = (current_time - self.last_time).to_sec()
		
		meas_3 = Float64(error)
		self.e.publish(meas_3)
		#print(current_time)
		#print(self.last_time)
		#print(dif_time)

		print("Kp="+ str(Kp))
		print("Ki="+ str(Ki))
		print("Kd="+ str(Kd))

		
		PID= Kp * error + Ki * self.aq_error * dif_time + Kd * (error - self.last_error) / dif_time
		PID = int(round(PID))
		self.last_time = current_time
		self.last_error = error
		str_val = UInt8()
		#print(PID)


		str_val.data = 100 + PID
		if str_val.data > 180:
			str_val.data= 180
		elif str_val.data < 0:
			str_val.data = 0
		#print(str_val)		
		self.str_pub.publish(str_val)
		


def main(args):

	rospy.init_node("Local_GPS_data") # have to define node for ROS
	
	control = PID_Controller() # call the class
	try:
		rospy.spin()
	except:
		print("Shutting down")


if __name__ == '__main__':
	main(sys.argv)

