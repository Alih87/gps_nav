#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, datetime
import os
from gps_nav.msg import coordinates
#from functools import partial

logs_path = r"/home/scout/boat_data/"

class logger_node(object):
	def __init__(self):
		self.content = []

	def log_data(self, data):
		x, y, theta = data.x, data.y, data.theta
		self.content.append(str(x)+","+str(y)+","+str(theta)+"\n")

	def optim_sub(self):
		rospy.init_node('data_logger')
		#get_logger_func = partial(log_data, fl)
		rospy.Subscriber('odom_pose', coordinates, self.log_data)
		rospy.sleep(0.01)

if __name__ == '__main__':
	logger = logger_node()
	dt = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M:%S")
	print("\n[INFO] Collecting Navigation data ...")
	logger.optim_sub()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
		logger.optim_sub()
		rate.sleep()

	with open(logs_path + "NAV_LOG_"+dt+".txt", "w") as f:
		for line in logger.content:
			f.write(line)
		f.close()
	print("\n[INFO] Data logged.")

