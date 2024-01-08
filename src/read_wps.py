#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys, os
from gps_nav.msg import coordinates, pose_xy, flag
from gps_nav.srv import wps_srv
from math import atan, atan2, pi

class send_wp_data():
	def __init__(self, path):
		self.path = path
		points = []

	def read_file(self):
		with open(self.path) as f:
			lines = f.readlines()
		for line in lines:
			lat, lon = map(float, lines.split(','))
			points.append((lat,lon))

if __name__=='__main__':
	home_dir = os.environ['HOME']
	path = home_dir + "/boat_data/wps/"

	wp_data = send_wp_data(path)

	print(wp_data.read_file())
