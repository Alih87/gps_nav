#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys, os
from gps_nav.msg import coordinates, pose_xy, flag
from gps_nav.srv import wps_srv, wps_srvResponse
from math import atan, atan2, pi

class send_wp_data():
	def __init__(self, path):
		self.path = path
		self.all_points_listed = False
		self.X, self.Y, self.Theta = [], [], []

	def read_file(self):
		with open(self.path) as f:
			lines = f.readlines()
		for line in lines:
			lat, lon = map(float, line.split('\n')[0].split(','))
			self.X.append(lat)
			self.Y.append(lon)
			self.Theta.append(0)
		assert len(self.X) == len(self.Y) ==len(self.Theta)
		self.all_points_listed = True

	def handle_wps_req(self, req):
		self.read_file()
		if req.send and self.all_points_listed:
			return wps_srvResponse(self.X, self.Y, self.Theta)

	def wps_server(self):
		rospy.init_node('wps_service')
		s = rospy.Service('wps_srv', wps_srv, self.handle_wps_req)
		rospy.spin()

if __name__ == '__main__':
	home_dir = os.environ['HOME']
	path = home_dir + "/boat_data/wps/wp_data.txt"

	wp_data = send_wp_data(path)
	
	wp_data.wps_server()
