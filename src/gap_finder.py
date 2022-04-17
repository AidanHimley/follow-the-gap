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
		rospy.init_node('gap_finder', anonymous = True)
		rospy.Subscriber("/car_7/scan", LaserScan, self.callback)
		self.pub = rospy.Publisher('/car7/gap_info', gap_info, queue_size=10)

		# Some useful variable declarations.
		self.ANGLE_RANGE = 240			# Hokuyo 4LX has 240 degrees FoV for scan
		self.CAR_LENGTH = 0.50			# Traxxas Rally is 20 inches or 0.5 meters
		self.safety_radius = 0.1
		self.disparity_threshold = 0.3
		
		rospy.spin()

	def getRanges(self, data):
		"""data: single message from topic /scan
		angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
		Outputs length in meters to object with angle in lidar scan field of view\n
		Cleans NaNs etc"""

		ranges = []
		angles = []

		for i in range(len(data.ranges)):
			if (data.ranges[i] and data.ranges[i] < data.range_max):
				ranges.append(data.ranges[i])
				angles.append(math.degrees(data.angle_min + i*data.angle_increment))
		return np.array(ranges), np.array(angles)

	def disparityExtenderWidest(self, ranges, angles, angle_increment):
		widest_gap_width = 0
		widest_gap_start = np.argmin(abs(angles))
		latest_gap_start = 0
		disparities = []
		for i in range(1, len(ranges)):
			if abs(ranges[i]-ranges[i-1]) > self.disparity_threshold:
				disparity_index = i if ranges[i] < ranges[i-1] else i-1
				disparities.append(angles[disparity_index])
				delta_i = int((self.safety_radius/(ranges[disparity_index]+self.CAR_LENGTH/2))/angle_increment)
				disparity_start = max(0, disparity_index-delta_i)
				if disparity_start-latest_gap_start > widest_gap_width:
					widest_gap_width = disparity_start - latest_gap_start
					widest_gap_start = latest_gap_start
				latest_gap_start = max(disparity_index + delta_i + 1, latest_gap_start)
		# check gap at end of array because it doesn't end in a disparity
		if len(ranges) - latest_gap_start > widest_gap_width:
			widest_gap_width = len(ranges) - latest_gap_start
			widest_gap_start = latest_gap_start
		rospy.loginfo("Seeing " + str(len(disparities)) + " disparities at angles:\n" + str(disparities))
		target_index = widest_gap_start + np.argmax(ranges[widest_gap_start : widest_gap_start+widest_gap_width])
		return angles[target_index], widest_gap_width*math.degrees(angle_increment), ranges[target_index]
	
	def disparityExtenderDeepest(self, ranges, angles, angle_increment):
		disparities = []
		for i in range(1, len(ranges)):
			if abs(ranges[i] - ranges[i-1]) > self.disparity_threshold:
				disparity_index = i if ranges[i] < ranges[i-1] else i-1
				delta_i = int((self.safety_radius/(ranges[disparity_index]+self.CAR_LENGTH/2))/angle_increment)
				disparities.append((disparity_index, delta_i))
		rospy.loginfo("Seeing " + str(len(disparities)) + " disparities at angles:\n" + str(disparities))
		for disparity_index, delta_i in disparities:
			for j in range(max(0, disparity_index-delta_i), min(len(ranges), disparity_index+delta_i+1)):
				ranges[j] = 0
		candidate_indices = (ranges==np.max(ranges)).nonzero()[0]
		target_index = candidate_indices[np.argmin(abs(angles[candidate_indices]))]		# lol
		width = 1
		i = target_index-1
		while i>=0 and ranges[i] != 0:
			width += 1
			i -= 1
		i = target_index+1
		while i<len(ranges) and ranges[i] != 0:
			width += 1
			i += 1
		return angles[target_index], width*math.degrees(angle_increment), ranges[target_index]

	def bubble(self, ranges, angles, angle_increment):
		disparities = []
		min_index = np.argmin(ranges)
		min_range = ranges[min_index]
		ang = math.arctan(self.safety_radius/min_range)
		rb = int(ang / angle_increment)
		for i in range(min_index - rb, min_index + rb + 1):
			ranges[i] = 0

	def callback(self, data):

		#-------------------gap-finding logic goes here------------------------
		rospy.loginfo("raw angles are " + str(data.angle_min) + " to " + str(data.angle_max))
		ranges, angles = self.getRanges(data)
		rospy.loginfo("angles are " + str(angles[0]) + " to " + str(angles[-1]))

		msg = gap_info()		# An empty msg is created of the type gap_info
		# msg.angle = ...			# position of the center of the selected gap
		# msg.width = ...			# width of the selected gap
		# msg.depth = ...			# depth of the selected gap
		msg.angle, msg.width, msg.depth = self.disparityExtenderWidest(ranges, angles, data.angle_increment)
		rospy.loginfo("Aiming for gap at " + str(msg.angle) + " with width " + str(msg.width) + " and depth" + str(msg.depth))
		self.pub.publish(msg)

if __name__ == '__main__':
	GapFinder()