#!/usr/bin/env python2
import sys
import math
import matplotlib.pyplot as plt
import numpy as np
from collections import namedtuple

import ros_numpy

from scipy import stats

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16, UInt8, UInt16

from time import localtime, strftime

wall_angle = 0
target_angle = wall_angle

def scan_callback(scan_msg):
    radius = np.asarray(scan_msg.ranges)
    # print(radius.dtype)
    # print(radius.shape)
    grid = OccupancyGrid()
    angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + scan_msg.angle_increment / 2, scan_msg.angle_increment)
    mask_fin = np.isfinite(radius)  # only consider finite radii

    mask = mask_fin

    masked_angles = angles[mask]
    masked_radius = radius[mask]

    # calculate coordinates of our masked values
    x = np.cos(masked_angles) * masked_radius
    y = np.sin(masked_angles) * masked_radius

    X = np.ones((np.size(x),3))
    X[:,0]=x
    X[:,2]=y

    points = np.column_stack((x, y))

    data = np.zeros((100,100),np.int8)
    for (x, val, y) in X:
        data[int(y*10)][int(x*10)]=100
    
    grid = ros_numpy.msgify(OccupancyGrid, data)
    grid.info.resolution=0.1
    # grid.info.origin=(Pose.Point=(50,50,0))
    # arr = arr.data  
    # grid.data = arr.ravel()
    # grid.info = MapMetaData()
    # grid.info.height = arr.shape[0]
    # grid.info.width = arr.shape[1]

    pub.publish(grid)

    

def main(args):
    global pub
    rospy.init_node("angle_calibration2134")
    try:
        rospy.Subscriber("/scan", LaserScan, scan_callback, queue_size=1)
        pub = rospy.Publisher("/OccGrid", OccupancyGrid, queue_size=1)
    except rospy.ROSInterruptException:
        pass
    rospy.spin()

if __name__ == '__main__':
	main(sys.argv)
