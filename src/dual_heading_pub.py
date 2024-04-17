#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial, sys
from gps_nav.msg import latlon_gps, heading_ang
from gps_nav.srv import gps_pos_srv, utm_srv, gps_pos_srvResponse
from math import atan2, pi
from utm import from_latlon

class dual_heading_node(object):
	def __init__(self):
		self.DGPS1, self.DGPS2 = [0, 0], [0, 0]
		self.DGPS1u, self.DGPS2u = [0, 0], [0, 0]
		self.CENTER = (388731.70, 3974424.49)
		self.ANGLE = 0
		self.prev_lat1, prev_long1 = 0, 0
	
	def get_utm_srv1(self, req):
		lat, long = req.lat, req.lon
		
		lati, loni, zo, ne = from_latlon(lat, long)
		if lati or loni:
			self.DGPS1 = [lati, loni]
			return gps_pos_srvResponse(True)
		else:
			return gps_pos_srvResponse(False)

	def get_utm_srv2(self, req):
		lat, long = req.lat, req.lon
		lati, loni, zo, ne = from_latlon(lat, long)
		if lati or loni:
			self.DGPS2 = [lati, loni]
			return gps_pos_srvResponse(True)
		else:
			return gps_pos_srvResponse(False)
	
	def gps_sub_Service1(self):
		#rospy.init_node('dgps_heading_angle', anonymous=False)
		gps_srv = rospy.Service('gps_pos_srv1', gps_pos_srv, self.get_utm_srv1)

	def gps_sub_Service2(self):
		#rospy.init_node('dgps_heading_angle', anonymous=False)
		gps_srv = rospy.Service('gps_pos_srv2', gps_pos_srv, self.get_utm_srv2)

	def heading_pub(self):
		#rospy.init_node('dgps_heading_angle', anonymous=False)
		pub = rospy.Publisher('dgps_heading', heading_ang, queue_size=1)

		try:
			#print(DGPS1)
			if self.DGPS1[0] == 0 and self.DGPS1[1] == 0:
				vec = (1, 1)
				pass
			else:
				DGPS1_m = from_latlon(self.DGPS1[0], self.DGPS1[1])
				DGPS2_m = from_latlon(self.DGPS2[0], self.DGPS2[1])
				#DGPS1_m = (self.DGPS1[0], self.DGPS1[1])
				#DGPS2_m = (self.DGPS2[0], self.DGPS2[1])
				self.DGPS1u[0], self.DGPS1u[1] = DGPS1_m[0], DGPS1_m[1]
				self.DGPS2u[0], self.DGPS2u[1] = DGPS2_m[0], DGPS2_m[1]
				self.DGPS1u[0], self.DGPS1u[1] = self.CENTER[0] - self.DGPS1u[0], self.CENTER[1] - self.DGPS1u[1]
				self.DGPS2u[0], self.DGPS2u[1] = self.CENTER[0] - self.DGPS2u[0], self.CENTER[1] - self.DGPS2u[1]
				vec = (self.DGPS2u[0] - self.DGPS1u[0], self.DGPS2u[1] - self.DGPS1u[1])
				#abs_vec = (vec[0]**2 + vec[1]**2)**0.5
				#unit_vec = (vec[0]/abs_vec, vec[1]/abs_vec)
					
		except ZeroDivisionError:
			sys.stdout.write("[INFO] Division by Zero!")
			unit_vec = (0, 0)

		#except OutOfRangeError:
		#	pass
		try:
			theta = atan2(vec[1], vec[0])*(180/pi) - 90
		except:
			theta = self.ANGLE
		if type(theta) is not None:
			self.ANGLE = theta
		self.ANGLE = (self.ANGLE + 180) % 360 - 180
		pub.publish(self.ANGLE)
		#rospy.sleep(0.002)

	def gps1_callback(self, data):
		if self.DGPS1 is not None:
			lati, longi = data.lat, data.lon
			if lati != 0.0 and longi != 0.0:
				self.prev_lat1, self.prev_long1 = lati, longi
				self.DGPS1 = [lati, longi]
			else:
				lati, longi = self.prev_lat1, self.prev_long1

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
	rospy.init_node('dgps_heading_angle', anonymous=False)
	rate = rospy.Rate(20)
	heading_obj = dual_heading_node()
	#heading_obj.gps_sub_Service1()
	#heading_obj.gps_sub_Service2()
	while not rospy.is_shutdown():
		heading_obj.gps1_sub()
		heading_obj.gps2_sub()
		heading_obj.heading_pub()
		rate.sleep()
