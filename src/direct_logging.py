#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys, datetime
from gps_nav.msg import latlon_gps, coordinates
from utm import from_latlon
from gps_nav.srv import cont_log_srv, gps_pos_srv, gps_pos_srvResponse, cont_log_srvResponse

class direct_log(object):
	def __init__(self):
		self.X, self.Y, self.theta = 0, 0, 0

	def get_utm_srv(self, req):
		lat, long = req.lat, req.lon
		self.X, self.Y, zo, ne = from_latlon(lat, long)
		if self.X or self.Y:
			return gps_pos_srvResponse(True)
		else:
			return gps_pos_srvResponse(False)


	def gps_sub_Service(self):
		#rospy.init_node('direct_log', anonymous=False)
		gps_srv = rospy.Service('gps_pos_srv1', gps_pos_srv, self.get_utm_srv)

	def cont_log_cb(self, req):
		if req.r:
			return cont_log_srvResponse(self.X, self.Y, self.theta)

	def contLoggingServer(self):
		#rospy.init_node('direct_log', anonymous=False)
		rospy.Service('cont_log', cont_log_srv, self.cont_log_cb)

	def get_utm(self, data):
		lat, long = data.lat, data.lon
		self.X, self.Y, zo, ne = from_latlon(lat, long)
		#self.X, self.Y = self.X - self.CENTER[0], self.Y - self.CENTER[1]
		#self.X, self.Y = self.X, self.Y
		#ZONE = str(zo)+ne
	
	def utm_pub(self):
		#rospy.init_node('direct_log', anonymous=False)
		pub = rospy.Publisher('odom_pose', coordinates, queue_size=1)
		pub.publish(self.X,self.Y,self.theta)

	def gps_sub(self):
		#rospy.init_node('direct_log', anonymous=False)
		rospy.Subscriber('gps_pos1', latlon_gps, self.get_utm)

if __name__=='__main__':
	rospy.init_node('direct_log', anonymous=False)
	rate = rospy.Rate(5)
	dl = direct_log()
	#dl.gps_sub_Service()
	dl.gps_sub()
	dl.contLoggingServer()
	rospy.spin()
