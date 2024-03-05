#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, latlon_gps, heading_ang
from gps_nav.srv import gps_pos_srv, utm_srv, gps_pos_srvResponse

class heading_KF(object):
	def __init__(self):
		

