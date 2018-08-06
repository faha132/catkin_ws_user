#!/usr/bin/env python

# --- imports ---
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import ros_numpy

# --- definitions ---
def resetGrid():
    global occupancy_grid
    for cell in xrange(len(occupancy_grid.data)):
        occupancy_grid.data[cell]=FREE
          # set all values to "FREE"

# to a given cartesian x,y coordinate, mark the corresponding cell in the grid as "OCCUPIED"
def setCell(x,y):
    global occupancy_grid

    res = occupancy_grid.info.resolution
    x_scaled = (x * 1.0 / res) + occupancy_grid.info.width/2
    y_scaled = (y * 1.0 / res) + occupancy_grid.info.height/2

    if x_scaled >= occupancy_grid.info.width or x_scaled < 0 or y_scaled >= occupancy_grid.info.height or y_scaled < 0:
        return

    offset = (int(round(x_scaled)) - 1) * occupancy_grid.info.height
    occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = OCCUPIED  


def scanCallback(scan_msg):
    global occupancy_grid

    resetGrid()

    radius = np.asarray(scan_msg.ranges)
    # print(radius.dtype)
    # print(radius.shape)
    angles = np.arange(scan_msg.angle_min, 
                       scan_msg.angle_max + scan_msg.angle_increment / 2, scan_msg.angle_increment)
    mask = np.isfinite(radius)  # only consider finite radii

    masked_angles = angles[mask]
    masked_radius = radius[mask]

    # calculate coordinates of our masked values
    x = np.cos(masked_angles) * masked_radius
    y = np.sin(masked_angles) * masked_radius

    X = np.ones((np.size(x),2))
    X[:,0]=x
    X[:,1]=y

    for (x, y) in X:
        setCell(x,y)
    #print(OccupancyGrid)
    #grid = ros_numpy.msgify(OccupancyGrid, data)
    #grid.info.resolution=0.1
    # convert scan measurements into an occupancy grid    
    pub_grid.publish(occupancy_grid)

# --- main --
FREE = -1
OCCUPIED = 100
rospy.init_node("scan_grid")

# init occupancy grid
occupancy_grid = OccupancyGrid()
# was "laser" got error in rviz
occupancy_grid.header.frame_id = "map"
occupancy_grid.info.resolution = 0.1 # in m/cell

# width x height cells
occupancy_grid.info.width = 1000
occupancy_grid.info.height = 1000
occupancy_grid.data = np.zeros(occupancy_grid.info.width * occupancy_grid.info.height)

# origin is shifted at half of cell size * resolution
occupancy_grid.info.origin.position.x = int(-1.0 * occupancy_grid.info.width / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.y = int(-1.0 * occupancy_grid.info.height / 2.0) * occupancy_grid.info.resolution
occupancy_grid.info.origin.position.z = 0
occupancy_grid.info.origin.orientation.x = 0
occupancy_grid.info.origin.orientation.y = 0
occupancy_grid.info.origin.orientation.z = 0
occupancy_grid.info.origin.orientation.w = 1

rospy.Subscriber("scan", LaserScan, scanCallback, queue_size=100)
pub_grid = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)

rospy.spin()
