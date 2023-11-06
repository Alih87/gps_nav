#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial, sys
from gps_nav.msg import latlon_gps, heading_ang
from math import atan2, pi
from utm import from_latlon

DGPS1, DGPS2 = [0, 0], [0, 0]
DGPS1u, DGPS2u = [0, 0], [0, 0]
CENTER = (388731.70, 3974424.49)
ANGLE = 0

def heading_pub():
	rospy.init_node('dgps_heading_angle', anonymous=False)
	pub = rospy.Publisher('dgps_heading', heading_ang, queue_size=30)
	global DGPS1, DGPS2, ANGLE

	try:
		#print(DGPS1)
		if DGPS1[0] == 0 and DGPS1[1] == 0:
			vec = (1, 1)
			pass
		else:
			DGPS1_m = from_latlon(DGPS1[0], DGPS1[1])
			DGPS2_m = from_latlon(DGPS2[0], DGPS2[1])
			DGPS1u[0], DGPS1u[1] = DGPS1_m[0], DGPS1_m[1]
			DGPS2u[0], DGPS2u[1] = DGPS2_m[0], DGPS2_m[1]
			#DGPS1u[0], DGPS1u[1] = CENTER[0] - DGPS1u[0], CENTER[1] - DGPS1u[1]
			#DGPS2u[0], DGPS2u[1] = CENTER[0] - DGPS2u[0], CENTER[1] - DGPS2u[1]
			vec = (DGPS2u[0] - DGPS1u[0], DGPS2u[1] - DGPS1u[1])
			#abs_vec = (vec[0]**2 + vec[1]**2)**0.5
			#unit_vec = (vec[0]/abs_vec, vec[1]/abs_vec)
			
	except ZeroDivisionError:
		sys.stdout.write("[INFO] Division by Zero!")
		unit_vec = (0, 0)

	#except OutOfRangeError:
	#	pass

	theta = atan2(vec[1], vec[0])*(180/pi)
	if type(theta) is not None:
		ANGLE = theta
	#print(theta)
	pub.publish(ANGLE)
	rospy.sleep(0.01)

def gps1_callback(data):
	global DGPS1
	if DGPS1 is not None:
		lati, longi = data.lat, data.lon
		DGPS1 = [lati, longi]

def gps2_callback(data):
	global DGPS2
	if DGPS2 is not None:
		lati, longi = data.lat, data.lon
		DGPS2 = [lati, longi]

def gps1_sub():
	rospy.init_node('dgps_heading_angle', anonymous=False)
	rospy.Subscriber('gps_pos1', latlon_gps, gps1_callback)

def gps2_sub():
	rospy.init_node('dgps_heading_angle', anonymous=False)
	rospy.Subscriber('gps_pos2', latlon_gps, gps2_callback)

if __name__ == '__main__':
	while not rospy.is_shutdown():
		gps1_sub()
		gps2_sub()
		heading_pub()

