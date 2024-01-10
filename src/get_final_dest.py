#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, flag, table1
from gps_nav.srv import wps_srv
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

	def done_callback(self, data):
		if data.flag:
			self.idx += 1
		else:
			pass
		
	def publish_curr_final_pos(self):
		rospy.init_node("current_final_pos", anonymous=False)
		pub = rospy.Publisher("final_pos", coordinates, queue_size=10)
		pub.publish(self.x[self.idx], self.y[self.idx], self.theta[self.idx])

	def publish_dest_wp(self):
		rospy.init_node("current_final_pos", anonymous=False)
		pub = rospy.Publisher("wp_table1", table1, queue_size=10)
		pub.publish(self.wp_ls[0], self.wp_ls[1], self.wp_ls[2], self.wp_ls[3])

	def complete_flag_sub(self):
		rospy.init_node('current_final_pos', anonymous=False)
		rospy.Subscriber("done_flag", flag, self.done_callback)
		rospy.sleep(0.01)

class wpsService():
	def __init__(self):
		self.x, self.y, self.theta = [], [], []

	def get_wps_from_srv(self):
		rospy.wait_for_service('wps_service')
		wps = rospy.ServiceProxy('wps_service', wp_srv)
		try:
			self.points = wps(True)
			print(self.points)
		except rospy.ServiceException as exc:
			print("Service did not process request: " + str(exc))

if __name__ == '__main__':
	#wp_1 = rospy.get_param("/dest_pos_pub/wp_1")
	#wp_2 = rospy.get_param("/dest_pos_pub/wp_2")
	#wp_3 = rospy.get_param("/dest_pos_pub/wp_3")
	#wp_4 = rospy.get_param("/dest_pos_pub/wp_4")

	#CENTER = (388731.70, 3974424.49)
	#ls = [wp_1, wp_2, wp_3, wp_4]

	wps_response = wpsService()
	wps_response.get_wps_from_srv()	

	x, y = [], []
	for s in ls:
		X, Y = wps_response.points.split(',')
		x.append(float(X))
		y.append(float(Y))
		theta.append(float(0))
	
	try:
		assert len(x) == len(y) and len(y) == len(theta) and len(x) == len(theta)
	except:
		print("[INFO] Assertion Failed. Check number of destination TM coordinates.")
		while(True):
			pass
	idx = 0
	dests_obj = get_final_dests(x, y, theta, ls, idx)
	print("\n[ INFO] Publishing destination information ...\n")
	while not rospy.is_shutdown():
		if dests_obj.idx < len(ls):
			dests_obj.publish_curr_final_pos()
			dests_obj.publish_dest_wp()
			dests_obj.complete_flag_sub()
		else:
			break
		#if idx >= len(x):
		#	print("\n[INFO] Final destination reached.")
		#	break
