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
		self.disparity_threshold = 0.25
		
		rospy.spin()

	def getRanges(self, data):
		"""data: single message from topic /scan
		angle: between -30 to 210 degrees, where 0 degrees is directly to the right, and 90 degrees is directly in front
		Outputs length in meters to object with angle in lidar scan field of view\n
		Cleans NaNs etc"""

		ranges = []
		angles = []

		for i in range(len(data.ranges)):
			if (data.ranges[i] and data.ranges[i] < data.range_max and data.ranges[i] > data.range_min):
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
		for gap_start in range(1, len(ranges)):
			if abs(ranges[gap_start] - ranges[gap_start-1]) > self.disparity_threshold:
				disparity_index = gap_start if ranges[gap_start] < ranges[gap_start-1] else gap_start-1
				delta_i = int((self.safety_radius/(ranges[disparity_index]+self.CAR_LENGTH/2))/angle_increment)
				disparities.append((disparity_index, delta_i))
		disparity_angles = [angles[disparity[0]] for disparity in disparities]
		rospy.loginfo("Seeing " + str(len(disparities)) + " disparities at angles:\n" + str(disparity_angles))
		
		# if no disparities, use bubble algorithm instead
		if len(disparities) == 0:
			return self.bubble(ranges, angles, angle_increment)
		
		for disparity_index, delta_i in disparities:
			for j in range(max(0, disparity_index-delta_i), min(len(ranges), disparity_index+delta_i+1)):
				ranges[j] = 0
		# rospy.loginfo(ranges)

		# find deepest gap, closest to straight ahead as tiebreaker
		candidate_indices = (ranges==np.max(ranges)).nonzero()[0]
		deepest_index = candidate_indices[np.argmin(abs(angles[candidate_indices]))]		# lol
		
		# find start and end of gap
		gap_start = deepest_index
		while gap_start>=0 and ranges[gap_start] != 0:
			gap_start -= 1
		gap_end = deepest_index+1
		while gap_end<len(ranges) and ranges[gap_end] != 0:
			gap_end += 1
		
		# aim for the center of the gap, calculate width
		target_index = gap_start + int((gap_end-gap_start)/2)
		width=gap_end-gap_start
		return angles[target_index], width*math.degrees(angle_increment), ranges[deepest_index]

	def bubble(self, ranges, angles, angle_increment):
		min_index = np.argmin(ranges)
		min_range = ranges[min_index]
		delta_i = int((self.safety_radius/(min_range+self.CAR_LENGTH/2))/angle_increment)
		print("avoiding obstacle at angle " + str(angles[min_index]))
		if angles[min_index] > 0:
			target_index = np.argmax(ranges[0: min_index-delta_i])
			width = min_index-delta_i
		else:
			target_index = min_index+delta_i + np.argmax(ranges[min_index+delta_i:])
			width = len(angles) - (min_index+delta_i)
		return angles[target_index], width*math.degrees(angle_increment), ranges[target_index]

	def callback(self, data):

		#-------------------gap-finding logic goes here------------------------
		# rospy.loginfo("raw angles are " + str(data.angle_min) + " to " + str(data.angle_max))
		ranges, angles = self.getRanges(data)
		start = np.argmin(np.abs(angles+90))
		end = np.argmin(np.abs(angles-90))
		ranges, angles, = ranges[start:end], angles[start:end]
		# rospy.loginfo("angles are " + str(angles[0]) + " to " + str(angles[-1]))

		msg = gap_info()		# An empty msg is created of the type gap_info
		# msg.angle = ...			# position of the center of the selected gap
		# msg.width = ...			# width of the selected gap
		# msg.depth = ...			# depth of the selected gap
		msg.angle, msg.width, msg.depth = self.disparityExtenderDeepest(ranges, angles, data.angle_increment)
		# for i in range(len(angles)):
		# 	print("angle: " + str(angles[i]) + "\trange: " + str(ranges[i]))
		# print(len(angles))
		rospy.loginfo("Aiming for gap at " + str(msg.angle) + " with width " + str(msg.width) + " and depth" + str(msg.depth))
		self.pub.publish(msg)

if __name__ == '__main__':
	GapFinder()