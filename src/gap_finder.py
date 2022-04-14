#!/usr/bin/env python
import rospy
import math
from sensor_msgs.msg import LaserScan
from gap.msg import gap_info
import numpy as np

class GapFinder():

	def __init__(self):
		print("Hokuyo LIDAR node started")
		
		# initialize node, subscriber, and publisher
		rospy.init_node('car7/gap_finder', anonymous = True)
		rospy.Subscriber("/car_7/scan", LaserScan, self.callback)
		self.pub = rospy.Publisher('car7/error', gap_info, queue_size=10)

		# Some useful variable declarations.
		self.ANGLE_RANGE = 240			# Hokuyo 4LX has 240 degrees FoV for scan
		self.CAR_LENGTH = 0.50			# Traxxas Rally is 20 inches or 0.5 meters
		self.safety_radius = 0.5
		self.disparity_threshold = 0.5
		
		rospy.spin()

	def getRanges(self, data):
		"""data: single message from topic /scan
		angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
		Outputs length in meters to object with angle in lidar scan field of view\n
		Cleans NaNs etc"""

		ranges = np.array(data.ranges)


		for i in range(len(ranges)):
			if not (ranges[i] and ranges[i] < data.range_max):
				ranges[i] = data.range_max
		return ranges

	def disparityExtender(self, ranges):
		for i in range(len(ranges)):
			pass

	def callback(self, data):

		#-------------------gap-finding logic goes here------------------------
		ranges = self.getRanges(data)
		angles = np.array([math.degrees(data.angle_min + i*data.angle_increment) for i in len(ranges)]) - 90

		msg = gap_info()		# An empty msg is created of the type gap_info
		msg.angle = ...			# position of the center of the selected gap
		msg.width = ...			# width of the selected gap
		msg.depth = ...			# depth of the selected gap
		self.pub.publish(msg)

if __name__ == '__main__':
	GapFinder()
