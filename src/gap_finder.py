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
		self.safety_radius = 0.25
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

	def disparityExtenderWidest(self, ranges, angles, angle_increment):
		widest_gap_width = 0
		widest_gap_start = np.argmin(abs(angles))
		latest_gap_start = 0
		for i in range(1, len(ranges)):
			if abs(ranges[i]-ranges[i-1]) > self.disparity_threshold:
				disparity_index = i if ranges[i] < ranges[i-1] else i-1
				delta_i = int((self.safety_radius/ranges[disparity_index])/angle_increment)
				disparity_start = max(0, disparity_index-delta_i)
				if disparity_start-latest_gap_start > widest_gap_width:
					widest_gap_width = disparity_start - latest_gap_start
					widest_gap_start = latest_gap_start
				latest_gap_start = disparity_index + delta_i + 1
		target_index = np.argmax(ranges[widest_gap_start : widest_gap_start+widest_gap_width])
		return angles[target_index], widest_gap_width*angle_increment, ranges[target_index]
	
	def disparityExtenderDeepest(self, ranges, angles, angle_increment):
		for i in range(1, len(ranges)):
			if abs(ranges[i] - ranges[i-1]) > self.disparity_threshold:
				disparity_index = i if ranges[i] < ranges[i-1] else i-1
				delta_i = int((self.safety_radius/ranges[disparity_index])/angle_increment)
				for j in range(max(0, disparity_index-delta_i), min(len(ranges), disparity_index+delta_i)):
					ranges[j] = 0
		candidate_indices = (ranges==np.max(ranges)).nonzero()[0]
		target_index = candidate_indices[np.argmin(abs(angles[candidate_indices]))]		# lol
		width = 1
		i = target_index-1
		while ranges[i] != 0 and i>=0:
			width += 1
			i -= 1
		i = target_index+1
		while ranges[i] != 0 and i<len(ranges):
			width += 1
			i += 1
		return angles[target_index], width*angle_increment, ranges[target_index]

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
