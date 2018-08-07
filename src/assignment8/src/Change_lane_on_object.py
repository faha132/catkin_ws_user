#!/usr/bin/env python
import numpy as np
import sys
import rospy
import rospkg
import math
import tf

from std_msgs.msg import Int32, UInt8, Float64, Int16, Float64MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#from assignment7 import field_Controller
from objectDetection import obst_detector
from nav_msgs.msg import Odometry


class obst_driver:

	def __init__(self):
		rospy.on_shutdown(self.shutdown)
		self.shutdown_ = False
                #publisher
		self.obstacle_pub = rospy.Publisher("/obstacle/warning", Int32, queue_size=1)
		self.vel_pub = rospy.Publisher("/fabiankhaled/manual_control/speed", 
                                 Int16, queue_size=100)
		self.str_pub = rospy.Publisher("/fabiankhaled/steering", UInt8, queue_size=1)
		self.lane_change = rospy.Publisher("/Lane_change", Bool, queue_size=1)
                #imageimport
		self.bridge = CvBridge()
		# Field_Controller
		self.resolution = 10  # cm
		self.lane = 0
		# driving forward
		self.speed_value = 1000  # 2000
		# load forcefield
		rospack = rospkg.RosPack()
		self.file_path = rospack.get_path('assignment8') + '/src/'
		if (self.lane == 0):
			self.matrix = np.load(self.file_path + 'matrix100cm_lane1.npy')
		else:
			self.matrix = np.load(self.file_path + 'matrix100cm_lane2.npy')
		# dynamic mapsize for some reason matrix is not y * x (rows by columns)
		self.map_size_x, self.map_size_y = self.matrix.shape[:2]
		self.map_size_y *= self.resolution
		self.map_size_x *= self.resolution
		# PID controller intern variables
		self.aq_error = 0.
		self.last_error = 0.
		self.last_time = rospy.Time.now()
		self.str_val = 0
                #object avoiding
		self.time_acc = rospy.Time.now()
                #subscribers
		self.image_sub = rospy.Subscriber("/fabiankhaled/app/camera/depth/image_raw",
                                    Image, self.obstacle_callback, queue_size=1)
		self.obstacle_sub = rospy.Subscriber("/obstacle/warning", Int32, 
                                       self.callback, queue_size=1)
		self.odom_sub = rospy.Subscriber("/localization/odom/0", Odometry, 
                                   self.PID_call_back)
		self.lane_change_sub = rospy.Subscriber("/Lane_change", Bool, 
                                          self.Lane_cahange)

	def obstacle_callback(self, data):
		try:
		  depth_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
		except CvBridgeError as e:
		  print(e)
                #numpyfy image
		depth_array = np.array(depth_image, dtype=np.uint16)
		height, width = depth_array.shape[0:2]
                #most far distance is about 500
		switch = 5000
		# only corners and centers in ROI
		#dynamic asuming x is from left to right
		shiftBy = self.str_val // 180 
		for y in xrange(height//3, 2*(height//3), height // 12 - 1):
		  for x in xrange(shiftBy*(width//180), shiftBy*(width//180)+(width//3), 
                    width // 12 - 1):
			if depth_array[y, x] >= 60:
		        # mounted objects have closer range
			  switch = min(depth_array[y, x], switch)

		if not self.shutdown_:
			try:
				# 0 would be inside the camera -> flag for nothing detected
				self.obstacle_pub.publish(Int32(switch))
			except CvBridgeError as e:
				print(e)

	def Lane_cahange(self, raw_msgs):
		# swap out loaded force field
		self.lane = (self.lane + 1) % 2
		if (self.lane == 0):
			self.matrix = np.load(self.file_path + 'matrix100cm_lane1.npy')
			print("Lane_1")
		else:
			self.matrix = np.load(self.file_path + 'matrix100cm_lane2.npy')
			print("Lane_2")
		self.map_size_x, self.map_size_y = self.matrix.shape[
			:2]  # *self.resolution #cm
		self.map_size_y *= self.resolution
		self.map_size_x *= self.resolution

	def PID_call_back(self, raw_msgs):
		# retrieving values from message
		x = raw_msgs.pose.pose.position.x
		y = raw_msgs.pose.pose.position.y
		orientation = raw_msgs.pose.pose.orientation
		orientation_array = [orientation.x,
			orientation.y, orientation.z, orientation.w]
		# change the read-out data from Euler to Rad
		orientation_in_Rad = tf.transformations.euler_from_quaternion(
			orientation_array)
		yaw = orientation_in_Rad[2]
		x_ind = np.int(x * self.resolution)
		y_ind = np.int(y * self.resolution)
		# if abroad, car virtually set on the map
		if x_ind < 0:
			x_ind = 0
		if x_ind > self.map_size_x / self.resolution - 1:
			# from some code on github
			# i wonder why it is not x_ind = self.map_size_x - 1
			x_ind = self.map_size_x / self.resolution - 1
		if y_ind < 0:
			y_ind = 0
		if y_ind > self.map_size_y / self.resolution - 1:
			y_ind = self.map_size_y / self.resolution - 1
		x_map, y_map = self.matrix[x_ind, y_ind]
		# if we imagine the odometry as polar coordinates
		# we have some radius r and a angle phi
		# so now we combine the steering angle given from the force field alpha and phi
		# with x = r * (cos(alpha-phi)) and y = (sin(alpha-phi))
		# maybe it was phi - alpha in both i wrote it on a paper
		# formula was given on the assignment sheet
		x_car = np.cos(yaw) * x_map + np.sin(yaw) * y_map
		y_car = -np.sin(yaw) * x_map + np.cos(yaw) * y_map
		# hyperparameters for PID
		Kp = 150
		Ki = 0
		Kd = 0.
		# so the error is the steepness of the correction anlge
		error = np.arctan(y_car / (x_car))
		# pseudo integral memory in borders
		self.aq_error = self.aq_error + error
		if self.aq_error > 10:
			self.aq_error = 10
		elif self.aq_error < -10:
			self.aq_error = -10
		# time between measurements for delta t
		current_time = rospy.Time.now()
		dif_time = (current_time - self.last_time).to_sec()
		# error manipulation with PI
		# + Kd * (error - self.last_error) / dif_time
		PID = Kp * error + Ki * self.aq_error * dif_time
		PID = int(round(PID))
		if (rospy.Time.now() - self.time_acc).to_sec() < 0.5 :
			if PID < -10:
				PID = -100
			else:
				PID = 80
		self.last_time = current_time
		self.last_error = error
		# detect min dist direction
		if (x_car < 0):
			vel = -self.speed_value
		else:
			vel = self.speed_value
		if x_car > 0:
			# reduce speed based on steering
			vel = max(self.speed_value, vel * ((np.pi / 3) / (abs(PID) + 1)))
			# valid steering angle -100<=alpha<=80
		if PID > 80:
			PID = 80
		elif PID < -100:
			PID = -100
# offset 100 == straight ahead
		PID = 100 + PID
		self.str_val = PID
		str_val = UInt8(PID)
		# dont start acceleration after shutdown
		if not self.shutdown_:
			self.str_pub.publish(str_val)
			self.vel_pub.publish(vel)

	def callback(self,raw_msg):
                #flag something detected
                if raw_msg.data != 5000:
                        if (rospy.Time.now() - self.time_acc).to_sec() > 3 :
                                self.lane_change.publish(Bool(True))
                                self.time_acc = rospy.Time.now()

	def shutdown(self):
		# set speed to 0
		print("shutdown!")
		self.shutdown_ = True
		self.vel_pub.publish(Int16(0))
		rospy.sleep(1)

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

