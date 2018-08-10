#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import sys
import rospy
import rospkg
import math
import tf
from matplotlib import pyplot as plt

from std_msgs.msg import Int32, UInt8, Float64, Int16, \
    Float64MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import cv2


class obst_driver:

    def __init__(self):
        self.bridge = CvBridge()
        rospy.on_shutdown(self.shutdown)
        self.shutdown_ = False

        # publisher

        self.image_pub_gray = \
            rospy.Publisher('/assignment34321/bin_img/gray', Image,
                            queue_size=1)

        # subscribers

        self.image_sub = \
            rospy.Subscriber('/fabiankhaled/app/camera/ir/image_raw',
                             Image, self.obstacle_callback,
                             queue_size=1)

    def obstacle_callback(self, data):

        # probably not good when there is sun

        ir_image = self.bridge.imgmsg_to_cv2(data, '16UC1')
        height, width = ir_image.shape[0:2]

        # dist camera to ground 155 in depth measure scale
        # cam image crop out car

        # hyperparams

        NOTROADCROP = 13 * height // 24
        TOPCROP = height // 24
        LANE_INTENSITY = 1000
        ir_image = ir_image[TOPCROP:NOTROADCROP, :]
        #update width height
        height, width = ir_image.shape[0:2]

        print height,width

        distance_radius = 5
        alpha = 11*np.pi / 24

        # rest

        x = width//2
        y = height
        score = [None,None,None]
        for direction in range(3):
            # 0 somewhat left 1 straight line 2 somewhat right
            dist_acc = 1
            det = False
            while not det:

        # ceil for val  != origin
        # subtraction y axis because numpy has 0,0 left upper corner

                x2 = int(math.ceil(x + math.cos(np.pi / 2
                         - alpha * (direction-1)) * dist_acc))
                y2 = int(math.floor(y - (math.sin(np.pi / 2
                         - alpha * (direction-1)) * dist_acc)))
                    
                dist_acc += 1
                if 0 < x2 and x2 < width and 0 < y2 and y2 < height:
                    if ir_image[y2, x2] > LANE_INTENSITY:
                        det = True
                else:
                        dist_acc = 0
                    break
            score[direction] = dist_acc
        print score
        self.image_pub_gray.publish(self.bridge.cv2_to_imgmsg(ir_image,
                                    '16UC1'))

    def shutdown(self):

        # set speed to 0

        print 'shutdown!'
        self.shutdown_ = True

        # self.vel_pub.publish(Int16(0))

        rospy.sleep(1)

    
def main(args):
    rospy.init_node('obst_driver', anonymous=True)
    driv = obst_driver()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print 'Shutting down'
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)

