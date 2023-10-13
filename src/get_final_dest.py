#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, flag
#from sbg_driver.msg import SbgGpsPos, SbgMag
from math import atan, pi

def user_input(xin, yin, theta_in):
    global x, y, theta
    x = map(int, xin)
    y = map(int, yin)
    theta = map(float, theta_in)

def done_callback(data):
	global idx
	if data.flag:
		idx += 1
	else:
		pass
	
def publish_curr_final_pos(x, y, theta, i):
    rospy.init_node("current_final_pos", anonymous=False)
    pub = rospy.Publisher("final_pos", coordinates, queue_size=10)
    pub.publish(x[i], y[i], theta[i])

def complete_flag_sub():
	rospy.init_node('current_final_pos', anonymous=False)
	rospy.Subscriber("done_flag", flag, done_callback)
	rospy.sleep(0.01)

if __name__ == '__main__':
	wp_1 = rospy.get_param("/dest_pos_pub/wp_1")
	wp_2 = rospy.get_param("/dest_pos_pub/wp_2")
	wp_3 = rospy.get_param("/dest_pos_pub/wp_3")
	wp_4 = rospy.get_param("/dest_pos_pub/wp_4")

	x, y, theta = [],[],[]
	idx = 0
	
	ls = [wp_1,wp_2,wp_3,wp_4]
	for s in ls:
		X, Y = s.split(',')
		x.append(float(X))
		y.append(float(Y))
		theta.append(float(0))
	#user_input(easting, northing, heading)
	try:
		assert len(x) == len(y) and len(y) == len(theta) and len(x) == len(theta)
	except:
		print("[INFO] Assertion Failed. Check number of destination TM coordinates.")
		while(True):
			pass
	print("\n[ INFO] Publishing destination information ...\n")
	while not rospy.is_shutdown():
		publish_curr_final_pos(x, y, theta, idx)
		complete_flag_sub()
		if idx >= len(x):
			print("\n[INFO] Final destination reached.")
			break
