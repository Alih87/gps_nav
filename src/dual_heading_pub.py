#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial, sys
from gps_nav.msg import latlon_gps, heading_ang
from math import atan2, pi

DGPS1, DGPS2 = (0, 0), (0, 0)
ANGLE = 0

def heading_pub():
	rospy.init_node('dgps_heading_angle', anonymous=False)
	pub = rospy.Publisher('dgps_heading', heading_ang, queue_size=30)
	global DGPS1, DGPS2, ANGLE

	try:
		vec = (DGPS2[0] - DGPS1[0], DGPS2[1] - DGPS1[1])
		abs_vec = (vec[0]**2 + vec[1]**2)**0.5
		unit_vec = (vec[0]/abs_vec, vec[1]/abs_vec)
	except ZeroDivisionError:
		sys.stdout.write("[INFO] Division by Zero!")
		unit_vec = (0, 0)

	theta = atan2(unit_vec[1], unit_vec[0])*(pi/180)
	if type(theta) is not None:
		ANGLE = theta
	#print(theta)
	pub.publish(ANGLE)
	rospy.sleep(0.01)

def gps1_callback(data):
	global DGPS1
	if DGPS1 is not None:
		DGPS1[0], DGPS1[1] = data.lat, data.lon

def gps2_callback():
	global DGPS2
	if DGPS2 is not None:
		DGPS2[0], DGPS2[1] = data.lat, data.lon

def gps1_sub():
	rospy.init_node('dgps_heading_angle', anonymous=False)
	rospy.Subscriber('gps_pub1', latlon_gps, gps1_callback)

def gps2_sub():
	rospy.init_node('dgps_heading_angle', anonymous=False)
	rospy.Subscriber('gps_pub2', latlon_gps, gps2_callback)

if __name__ == '__main__':
	gps1_sub()
	gps2_sub()
	heading_pub()

