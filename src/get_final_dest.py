#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy
from sbg_driver.msg import SbgGpsPos, SbgMag
from math import atan, pi

x, y, theta = [], [], []
idx = 0

def user_input(xin, yin, theta_in):
    global x, y, theta
    x = map(int, xin)
    y = map(int, yin)
    theta = map(float, theta_in)

def done_callback(data):
	global idx
	if bool(data):
		idx += 1
	else:
		pass
	
def publish_curr_final_pos(i):
    rospy.init_node("current_final_pos", anonymous=False)
    pub = rospy.Publisher("final_pos", pose_xy, queue_size=10)
    pub.publish(x[i], y[i], theta[i])

def complete_flag_sub():
	rospy.init_node('current_final_pos', anonymous=False)
	rospy.Subscriber("done_flag", flag, done_callback)
	rospy.sleep(0.01)

if __name__ == '__main__':
	#easting = rospy.get_param("/dest_pos_pub/easting")
	#northing = rospy.get_param("/dest_pos_pub/northing")
	#heading = rospy.get_param("/dest_pos_pub/heading")
	user_input(easting, northing, heading)
	try:
		assert len(x) == len(y) and len(y) == len(theta) and len(x) == len(theta)
	except:
		print("[INFO] Assertion Failed. Check number of destination TM coordinates.")
		while(True)
	print("\n[ INFO] Publishing destination information ...\n")
	while not rospy.is_shutdown():
		publish_curr_final_pos(i)
		complete_flag_sub()
		if idx => len(x):
			print("\n[INFO] Final destination reached.")
			break
