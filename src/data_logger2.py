#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, datetime
import os
from gps_nav.msg import coordinates
from gps_nav.srv import logging_srv, logging_srvResponse
#from functools import partial

home = os.environ['HOME']

logs_path = home+r"/boat_data/"

class logger_node(object):
	def __init__(self):
		self.content = []

	def log_data(self, req):
		x, y, theta, time = req.x, req.y, req.theta, req.time
		if (x and y and theta and time) or len(self.content) == 0:	
			self.content.append(str(x)+","+str(y)+","+str(theta)+","+str(time)+"\n")
			return logging_srvResponse(True)
		else:
			return logging_srvResponse(False)

	def optim_sub(self):
		rospy.init_node('data_logger2')
		lg_srv = rospy.Service('logg_srv', logging_srv, self.log_data)
		rospy.sleep(0.01)

if __name__ == '__main__':
	logger = logger_node()
	dt = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M:%S")
	print("\n[INFO] Collecting Navigation data ...")
	logger.optim_sub()
	rate = rospy.Rate(0.99)
	while not rospy.is_shutdown():
		rate.sleep()
	with open(logs_path + "NAV_LOG_"+dt+".txt", "w") as f:
		for line in logger.content:
			f.write(line)
		f.close()
	print("\n[INFO] Data logged.")

