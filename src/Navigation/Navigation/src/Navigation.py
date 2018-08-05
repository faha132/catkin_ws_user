#!/usr/bin/env python2

import rospy
import tf
import numpy as np
import rospkg
import math
import sys

from nav_msgs.msg import Odometry
from std_msgs.msg import UInt8, Float64, Int16, Float64MultiArray


class field_Controller:

    def __init__(self):
        rospy.init_node("field_Controller") # have to define node for ROS
        
        self.yaw= 0. 
        self.Stop= 0
        #self.setpoint= math.pi

        
        self.resolution = 10 # cm
        self.lane=1
        self.speed_value= 1300 #2000
        
        rospy.on_shutdown(self.shutdown)
        self.shutdown_=False

        rospack = rospkg.RosPack()
        self.file_path=rospack.get_path('Navigation')+'/src/'
        #self.matrix = np.load(file_path+'matrix100cm_lane2.npy')

        self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')
        
        #print(self.matrix.shape)
        self.map_size_x, self.map_size_y=self.matrix.shape[:2]#*self.resolution #cm
        self.map_size_y *= self.resolution
        self.map_size_x *= self.resolution

        #print("mapsize")
        #print(self.map_size_y,self.map_size_x)


        # PID controller variables
        self.aq_error= 0.
        self.last_error= 0.
        self.last_time = rospy.Time.now()


        self.dummya_1 = rospy.Subscriber("/localization/odom/0", Odometry, self.call_back)  #subscribe to topic
        self.dummya_2 = rospy.Subscriber("/Lane_change", Int16, self.Lane_cahange)

        
        self.vel_pub = rospy.Publisher("/fabiankhaled/manual_control/speed",Int16, queue_size=100)
        self.str_pub = rospy.Publisher("/fabiankhaled/steering",UInt8, queue_size=1)
        self.m_yaw = rospy.Publisher("/measured_yaw",Float64, queue_size=100)
        #self.set_point = rospy.Publisher("/set_point",Float64, queue_size=1)
    

    def Lane_cahange (self,raw_msgs):
        self.lane= raw_msgs.data
        
    
        #print("Stop: {0}".format(self.Stop))
        
        if (self.lane==1):
            self.matrix = np.load(self.file_path+'matrix100cm_lane1.npy')
            print("Lane_1")
        else:
            self.matrix = np.load(self.file_path+'matrix100cm_lane2.npy')
            
            print("Lane_2")

        self.map_size_x, self.map_size_y=self.matrix.shape[:2]#*self.resolution #cm     
        self.map_size_y *= self.resolution
        self.map_size_x *= self.resolution
            

    #    self.vel_pub.publish(vel)



    def call_back (self,raw_msgs):
        
        #rospy.loginfo(raw_msgs)
        x = raw_msgs.pose.pose.position.x
        y = raw_msgs.pose.pose.position.y
        orientation= raw_msgs.pose.pose.orientation
        
        #rospy.loginfo(orientation)
        orientation_array = [orientation.x, orientation.y, orientation.z, orientation.w]
        orientation_in_Rad= tf.transformations.euler_from_quaternion(orientation_array) #change the read-out data from Euler to Rad
        self.yaw =orientation_in_Rad[2]
        #print(type(self.yaw))
        
        x_ind = np.int(x*self.resolution)
        y_ind = np.int(y*self.resolution)


        if x_ind < 0:
            x_ind = 0

        if x_ind > self.map_size_x/self.resolution -1:
            x_ind = self.map_size_x/self.resolution -1

        if y_ind < 0:
            y_ind = 0

        if y_ind > self.map_size_y/self.resolution -1:
            y_ind = self.map_size_y/self.resolution -1

        x_map, y_map = self.matrix[x_ind, y_ind]
        x_car = np.cos(self.yaw)*x_map + np.sin(self.yaw)*y_map
        y_car = -np.sin(self.yaw)*x_map + np.cos(self.yaw)*y_map
        
        #meas_1 = Float64(self.yaw)
        #meas_2 = Float64(self.setpoint)
        #self.set_point.publish(meas_2)
        #self.m_yaw.publish(meas_1)
        
        #self.PID()


    #def PID(self):
        
        Kp= 85 #150 - 180
        Ki= 0.0002
        Kd= 0.00003
        
        error= np.arctan(y_car / (x_car))
        print(error)

        self.aq_error = self.aq_error + error
        
        if self.aq_error > 10:
            self.aq_error=10
        
        elif self.aq_error < -10:
            self.aq_error = -10

        current_time = rospy.Time.now()
        dif_time = (current_time - self.last_time).to_sec()

        #print(current_time)
        #print(self.last_time)
        #print(dif_time)

        
        PID= Kp * error  + Ki * self.aq_error * dif_time #+ Kd * (error - self.last_error) / dif_time
        PID = int(round(PID))
        self.last_time = current_time
        self.last_error = error
        #str_val = UInt8()
        #print(PID)

        #str_val.data = PID
        
        if (x_car<0):
            vel = -self.speed_value

        else:
            vel = self.speed_value


        
        if x_car > 0:
            vel = max(self.speed_value, vel * ((np.pi/3)/(abs(PID)+1)))

        

        if PID > 80:
            PID= 80
        elif PID < -100:
            PID = -100

        PID = 100 + PID
        print (PID)


        str_val = UInt8(PID) 
        #print(str_val)
        #print(self.shutdown_)
        if not self.shutdown_:
            #print(str_val)
            #print(vel)
            self.str_pub.publish(str_val)
            self.vel_pub.publish(vel)

    def shutdown(self):
        print("shutdown!")
        self.shutdown_=True
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
