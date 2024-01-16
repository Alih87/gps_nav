#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, flag, table1
from gps_nav.srv import wps_srv, flag_srv, flag_srvResponse
from math import atan, pi

class get_final_dests(object):
	def __init__(self, x, y, theta, wp_ls, idx):
		self.x, self.y, self.theta = x, y, theta
		self.wp_ls = wp_ls
		self.idx = idx
		self.points = []
	
	def user_input(self, xin, yin, theta_in):
		self.x = map(int, xin)
		self.y = map(int, yin)
		self.theta = map(float, theta_in)

	def done_callback(self, req):
		if req.cmd:
			self.idx += 1
			return flag_srvResponse(True)
		else:
			return flag_srvResponse(False)
		
	def publish_curr_final_pos(self):
		rospy.init_node("current_final_pos", anonymous=False)
		pub = rospy.Publisher("final_pos", coordinates, queue_size=1)
		pub.publish(self.x[self.idx], self.y[self.idx], self.theta[self.idx])

	def publish_dest_wp(self):
		rospy.init_node("current_final_pos", anonymous=False)
		pub = rospy.Publisher("wp_table1", table1, queue_size=1)
		pub.publish(self.wp_ls[0], self.wp_ls[1], self.wp_ls[2], self.wp_ls[3])

	#def complete_flag_sub(self):
	#	rospy.init_node('current_final_pos', anonymous=False)
	#	rospy.Subscriber("done_flag", flag, self.done_callback)
	
	def done_flag_server(self):
		rospy.init_node('current_final_pos')
		s = rospy.Service('done_flag_srv', flag_srv, self.done_callback)
		rospy.spin()

class wpsService():
	def __init__(self):
		self.x, self.y, self.theta = [], [], []

	def get_wps_from_srv(self):
		rospy.wait_for_service('wps_srv')
		wps = rospy.ServiceProxy('wps_srv', wps_srv)
		try:
			srv_resp = wps(True)
			self.x, self.y, self.theta = srv_resp.x, srv_resp.y, srv_resp.theta 
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

if __name__ == '__main__':
	wps_response = wpsService()
	wps_response.get_wps_from_srv()	

	x, y, theta = wps_response.x, wps_response.y, wps_response.theta
	
	try:
		assert len(x) == len(y) and len(y) == len(theta) and len(x) == len(theta)
	except:
		print("[INFO] Assertion Failed. Check number of destination TM coordinates.")
		while(True):
			pass
	ls = []
	for X, Y, TH in zip(x,y,theta):
		wp = str(X)+","+str(Y)
		ls.append(wp)

	idx = 0
	dests_obj = get_final_dests(x, y, theta, ls, idx)
	print("\n[ INFO] Publishing destination information ...\n")
	while not rospy.is_shutdown():
		if dests_obj.idx < len(ls):
			dests_obj.publish_curr_final_pos()
			dests_obj.publish_dest_wp()
			dests_obj.done_flag_server()
		else:
			break
		#if idx >= len(x):
		#	print("\n[INFO] Final destination reached.")
		#	break
