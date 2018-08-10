#!/usr/bin/env python
import numpy as np
import sys
import rospy
import rospkg
import math
import tf
from matplotlib import pyplot as plt

from std_msgs.msg import Int32, UInt8, Float64, Int16, Float64MultiArray, Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
import cv2


class obst_driver:

		def __init__(self):
			self.bridge=CvBridge()
			rospy.on_shutdown(self.shutdown)
			self.shutdown_ = False
			#publisher
			self.image_pub_gray = rospy.Publisher("/assignment34321/bin_img/gray",Image, queue_size=1	 )
			#subscribers
			self.image_sub = rospy.Subscriber("/fabiankhaled/app/camera/ir/image_raw",
											  Image, self.obstacle_callback, queue_size=1)

		def obstacle_callback(self, data):
			#probably not good when there is sun
			ir_image = self.bridge.imgmsg_to_cv2(data, "16UC1")
			height, width = ir_image.shape[0:2]
			#dist camera to ground 155 in depth measure scale
			#cam image crop out car
			NOTROADCROP = 29*height//48
			TOPCROP=height//24
			#between 5000-6000 good 5900
			LANE_INTENSITY = 6500
			ir_image = ir_image[TOPCROP:NOTROADCROP,:]
			mini,maxi = 10000,0
			acc = 0
			for y,row in enumerate(ir_image):
				for x, elem in enumerate(row):
					if elem > LANE_INTENSITY:
						ir_image[y,x]=62000
						#acc+=1
						#mini,maxi = min(mini,elem),max(maxi,elem)
					else:
						ir_image[y,x]=0
			#print(acc)
			self.image_pub_gray.publish(self.bridge.cv2_to_imgmsg(ir_image, "16UC1"))

		def shutdown(self):
			# set speed to 0
			print("shutdown!")
			self.shutdown_ = True
			#self.vel_pub.publish(Int16(0))
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

