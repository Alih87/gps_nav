#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial, sys
from gps_nav.msg import latlon_gps, heading_ang
from math import atan2, pi
from utm import from_latlon

class dual_heading_node(object):
	def __init__(self):
		self.DGPS1, self.DGPS2 = [0, 0], [0, 0]
		self.DGPS1u, self.DGPS2u = [0, 0], [0, 0]
		self.CENTER = (388731.70, 3974424.49)
		self.ANGLE = 0
	def heading_pub(self):
		rospy.init_node('dgps_heading_angle', anonymous=False)
		pub = rospy.Publisher('dgps_heading', heading_ang, queue_size=30)

		try:
			#print(DGPS1)
			if self.DGPS1[0] == 0 and self.DGPS1[1] == 0:
				vec = (1, 1)
				pass
			else:
				DGPS1_m = from_latlon(self.DGPS1[0], self.DGPS1[1])
				DGPS2_m = from_latlon(self.DGPS2[0], self.DGPS2[1])
				self.DGPS1u[0], self.DGPS1u[1] = DGPS1_m[0], DGPS1_m[1]
				self.DGPS2u[0], self.DGPS2u[1] = DGPS2_m[0], DGPS2_m[1]
				#DGPS1u[0], DGPS1u[1] = CENTER[0] - DGPS1u[0], CENTER[1] - DGPS1u[1]
				#DGPS2u[0], DGPS2u[1] = CENTER[0] - DGPS2u[0], CENTER[1] - DGPS2u[1]
				vec = (self.DGPS2u[0] - self.DGPS1u[0], self.DGPS2u[1] - self.DGPS1u[1])
				#abs_vec = (vec[0]**2 + vec[1]**2)**0.5
				#unit_vec = (vec[0]/abs_vec, vec[1]/abs_vec)
				
		except ZeroDivisionError:
			sys.stdout.write("[INFO] Division by Zero!")
			unit_vec = (0, 0)

		#except OutOfRangeError:
		#	pass
		try:
			theta = atan2(vec[1], vec[0])*(180/pi)
		except:
			theta = self.ANGLE
		if type(theta) is not None:
			self.ANGLE = theta
		#print(theta)
		pub.publish(self.ANGLE)
		rospy.sleep(0.01)

	def gps1_callback(self, data):
		if self.DGPS1 is not None:
			lati, longi = data.lat, data.lon
			self.DGPS1 = [lati, longi]

	def gps2_callback(self, data):
		if self.DGPS2 is not None:
			lati, longi = data.lat, data.lon
			self.DGPS2 = [lati, longi]

	def gps1_sub(self):
		rospy.init_node('dgps_heading_angle', anonymous=False)
		rospy.Subscriber('gps_pos1', latlon_gps, self.gps1_callback)

	def gps2_sub(self):
		rospy.init_node('dgps_heading_angle', anonymous=False)
		rospy.Subscriber('gps_pos2', latlon_gps, self.gps2_callback)

if __name__ == '__main__':
	heading_obj = dual_heading_node()
	while not rospy.is_shutdown():
		heading_obj.gps1_sub()
		heading_obj.gps2_sub()
		heading_obj.heading_pub()

