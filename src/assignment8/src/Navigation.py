#!/usr/bin/env python2

import rospy
import tf
import numpy as np
import rospkg
import math
import sys

from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Float64, Int16, Float64MultiArray, Bool


class field_Controller:

    def __init__(self):
        
        self.yaw= 0. 
        self.resolution = 10 # cm
        self.lane=0
        self.speed_value= 1000 #2000
        self.old_speed=0
        rospy.on_shutdown(self.shutdown)
        self.publishing=False
        rospack = rospkg.RosPack()
        self.file_path=rospack.get_path('Navigation')+'/src/'
        #load forcefield
        if (self.lane==0):
            self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')
        else:
            self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')
        #dynamic mapsize for some reason matrix is not y * x (rows by columns)
        self.map_size_x, self.map_size_y=self.matrix.shape[:2]
        self.map_size_y *= self.resolution
        self.map_size_x *= self.resolution
        # PID controller intern variables
        self.aq_error= 0.
        self.last_error= 0.
        self.last_time = rospy.Time.now()
        self.dummya_1 = rospy.Subscriber("/localization/odom/0", Odometry, self.call_back)
        self.vel_pub = rospy.Publisher("/fabiankhaled/manual_control/speed",
                Int16, queue_size=100)
        self.str_pub = rospy.Publisher("/fabiankhaled/steering",UInt8, queue_size=1)
        #listen on lane swap
        self.dummya_2 = rospy.Subscriber("/Lane_change", Bool, self.Lane_cahange)
        self.stop_sub = rospy.Subscriber("/nav/stop", Bool, self.callback_stop)

    def callback_stop(self, raw_msgs):
        #variable decides whether or not values are published
        self.publishing = raw_msgs.data

    def Lane_cahange (self,raw_msgs):
        #swap out loaded force field
            self.lane = (self.lane + 1) % 2
            if (self.lane==0):
                self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')
                print("Lane_1")
            else:
                self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')
                print("Lane_2")
            self.map_size_x, self.map_size_y=self.matrix.shape[:2]#*self.resolution #cm     
            self.map_size_y *= self.resolution
            self.map_size_x *= self.resolution

    def call_back(self,raw_msgs):
        #retrieving values from message
        x = raw_msgs.pose.pose.position.x
        y = raw_msgs.pose.pose.position.y
        orientation= raw_msgs.pose.pose.orientation
        orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
        #change the read-out data from Euler to Rad
        orientation_in_Rad= tf.transformations.euler_from_quaternion(orientation_array) 
        self.yaw =orientation_in_Rad[2]
        x_ind = np.int(x*self.resolution)
        y_ind = np.int(y*self.resolution)
        #if abroad, car virtually set on the map
        if x_ind < 0:
            x_ind = 0
        if x_ind > self.map_size_x/self.resolution -1:
            #from some code on github
            #i wonder why it is not x_ind = self.map_size_x - 1
            x_ind = self.map_size_x/self.resolution -1
        if y_ind < 0:
            y_ind = 0
        if y_ind > self.map_size_y/self.resolution -1:
            y_ind = self.map_size_y/self.resolution -1
        x_map, y_map = self.matrix[x_ind, y_ind]
        #if we imagine the odometry as polar coordinates 
        #we have some radius r and a angle phi
        #so now we combine the steering angle given from the force field alpha and phi
        #with x = r * (cos(alpha-phi)) and y = (sin(alpha-phi))
        #maybe it was phi - alpha in both i wrote it on a paper
        #formula was given on the assignment sheet
        x_car = np.cos(self.yaw)*x_map + np.sin(self.yaw)*y_map
        y_car = -np.sin(self.yaw)*x_map + np.cos(self.yaw)*y_map
        #hyperparameters for PID
        Kp= 150
        Ki= 0
        Kd= 0.
        #so the error is the steepness of the correction anlge
        error= np.arctan(y_car / (x_car))
        #pseudo integral memory in borders
        self.aq_error = self.aq_error + error
        if self.aq_error > 10:
            self.aq_error=10
        elif self.aq_error < -10:
            self.aq_error = -10
        #time between measurements for delta t
        current_time = rospy.Time.now()
        dif_time = (current_time - self.last_time).to_sec()
#error manipulation with PI
        PID= Kp * error  + Ki * self.aq_error * dif_time #+ Kd * (error - self.last_error) / dif_time
        PID = int(round(PID))
        self.last_time = current_time
        self.last_error = error
        #detect min dist direction
        if (x_car<0):
            vel = -self.speed_value
        else:
            vel = self.speed_value
        if x_car > 0:
            #reduce speed based on steering
            vel = max(self.speed_value, vel * ((np.pi/3)/(abs(PID)+1)))
            #valid steering angle -100<=alpha<=80 
        if PID > 80:
            PID= 80
        elif PID < -100:
            PID = -100
#offset 100 == straight ahead
        PID = 100 + PID
        str_val = UInt8(PID) 
        #dont start acceleration after shutdown
        if not self.publishing:
            self.str_pub.publish(str_val)
            self.vel_pub.publish(vel)

    def shutdown(self):
        #set speed to 0
        print("shutdown!")
        self.publishing=True
        self.vel_pub.publish(Int16(0))
        rospy.sleep(1)

def main(args):
    rospy.init_node("field_Controller") # have to define node for ROS
    control = field_Controller() # call the class
    try:
        rospy.spin()
    except:
        print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)