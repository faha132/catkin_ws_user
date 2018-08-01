#!/usr/bin/env python

# --- imports ---
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

# --- definitions ---
def resetGrid():
    global occupancy_grid
    for cell in xrange(len(occupancy_grid.data)):
        occupancy_grid.data[cell]=-1
          # set all values to "FREE"
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
    occupancy_grid.data[int(offset) + int(round(y_scaled) - 1)] = 100


def scanCallback(scan_msg):
    global occupancy_grid

    resetGrid()

    # convert scan measurements into an occupancy grid    
    
    radius = np.asarray(scan_msg.ranges)
    #array distance to object
    angles = np.arange(scan_msg.angle_min, scan_msg.angle_max + scan_msg.angle_increment / 2, scan_msg.angle_increment)
    #array mesuered angles
    mask = np.isfinite(radius)  # only consider finite radii

    #sort out angles where there was no object close enough

    masked_angles = angles[mask]
    masked_radius = radius[mask]

    # calculate coordinates of our masked values
    x = np.cos(masked_angles) * masked_radius
    y = np.sin(masked_angles) * masked_radius

    for pair in xrange(np.size(x)):
        setCell(x[pair],y[pair])

    #should apply mask for stationary objects build into the car

    pub_grid.publish(occupancy_grid)


# --- main ---
rospy.init_node("scan_grid")
# init occupancy grid
occupancy_grid = OccupancyGrid()
occupancy_grid.header.frame_id = "map"
#different frame id because rviz showed an error
#occupancy_grid.header.frame_id = "laser"
occupancy_grid.info.resolution = 0.05 # in m/cell
#occupancy_grid.info.resolution = None # in m/cell

# width x height cells
occupancy_grid.info.width = 100
#occupancy_grid.info.width = None
occupancy_grid.info.height = 100
#occupancy_grid.info.height = None
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
pub_grid = rospy.Publisher("scan_grid0", OccupancyGrid, queue_size=100)

rospy.spin()
