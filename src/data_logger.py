#!/usr/bin/env python3
import roslib; roslib.load_manifest('gps_nav')
import rospy, datetime
import os
from gps_nav.msg import pose_xy
#from functools import partial

logs_path = os.environ.get('HOME')+r"/boat_data/"
global CONTENT
CONTENT = []

def log_data(data):
    x, y, theta = data.x, data.y, data.theta
    CONTENT.append(str(x)+","+str(y)+","+str(theta)+"\n")

def optim_sub():
    rospy.init_node('data_logger')
    #get_logger_func = partial(log_data, fl)
    rospy.Subscriber('feedback', pose_xy, log_data)
    rospy.sleep(0.01)

if __name__ == '__main__':
	dt = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M:%S")
	print("\n[INFO] Collecting Navigation data ...")
	optim_sub()
	rate = rospy.Rate(1)
	while not rospy.is_shutdown():
	    optim_sub()
	    rate.sleep()

	with open(logs_path + "NAV_LOG_"+dt+".txt", "w") as f:
		for line in CONTENT:
			f.write(line)
		f.close()
	print("\n[INFO] Data logged.")

