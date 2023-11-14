#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, flag, table1
#from sbg_driver.msg import SbgGpsPos, SbgMag
from math import atan, pi

class get_final_dests():
	def __init__(self, wp_ls):
		self.idx = 0
		self.x, self.y, self.theta = [],[],[]
		self.wp_ls = wp_ls
		
	
	def user_input(self, xin, yin, theta_in):
		self.x = map(int, xin)
		self.y = map(int, yin)
		self.theta = map(float, theta_in)

	def done_callback(self, data):
		if data.flag:
			self.idx += 1
		else:
			pass
		
	def publish_curr_final_pos(self, x, y, theta):
		rospy.init_node("current_final_pos", anonymous=False)
		pub = rospy.Publisher("final_pos", coordinates, queue_size=10)
		pub.publish(x[self.idx], y[self.idx], theta[self.idx])

	def publish_dest_wp(self):
		rospy.init_node("current_final_pos", anonymous=False)
		pub = rospy.Publisher("wp_table1", table1, queue_size=10)
		pub.publish(wp_ls[0], wp_ls[1], wp_ls[2], wp_ls[3])

	def complete_flag_sub(self):
		rospy.init_node('current_final_pos', anonymous=False)
		rospy.Subscriber("done_flag", flag, self.done_callback)
		rospy.sleep(0.01)

if __name__ == '__main__':
	wp_1 = rospy.get_param("/dest_pos_pub/wp_1")
	wp_2 = rospy.get_param("/dest_pos_pub/wp_2")
	wp_3 = rospy.get_param("/dest_pos_pub/wp_3")
	wp_4 = rospy.get_param("/dest_pos_pub/wp_4")

	CENTER = (388731.70, 3974424.49)

	ls = [wp_1, wp_2, wp_3, wp_4]
	for s in ls:
		X, Y = s.split(',')
		x.append(float(X)-CENTER[0])
		y.append(float(Y)-CENTER[1])
		theta.append(float(0))
	#user_input(easting, northing, heading)
	
	try:
		assert len(x) == len(y) and len(y) == len(theta) and len(x) == len(theta)
	except:
		print("[INFO] Assertion Failed. Check number of destination TM coordinates.")
		while(True):
			pass
	
	dests_obj = get_final_dests(ls)

	print("\n[ INFO] Publishing destination information ...\n")
	while not rospy.is_shutdown():
		if idx < len(ls):
			self.publish_curr_final_pos(x, y, theta, idx)
			self.publish_dest_wp()
			self.complete_flag_sub()
		else:
			break
		#if idx >= len(x):
		#	print("\n[INFO] Final destination reached.")
		#	break
