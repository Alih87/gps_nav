#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, latlon_gps, heading_ang
#from sbg_driver.msg import SbgGpsPos, SbgMag
from nav_msgs.msg import Odometry
from utm import from_latlon
from math import atan, atan2, pi

# HEADING = deque(maxlen=5)
class gps_pose_node(object):
    def __init__(self):
        self.CENTER = (388731.70, 3974424.49)
        self.ZONE = ''
        self.X = 0
        self.Y = 0
        self.HEADING = 0

    def get_utm(self, data):
        lat, long = data.lat, data.lon
        self.X, self.Y, zo, ne = from_latlon(lat, long)
        self.X, self.Y = self.X - self.CENTER[0], self.Y - self.CENTER[1]
        self.X, self.Y = self.X, self.Y
        # ZONE = str(zo)+ne

    def get_heading(self, data):
        angle = data.angle
        self.HEADING = angle

    def gps_sub(self):
        rospy.init_node('gps_pose', anonymous=False)
        rospy.Subscriber('gps_pos1', latlon_gps, self.get_utm)
        rospy.sleep(0.01)

    def mag_sub(self):
        rospy.init_node('gps_pose', anonymous=False)
        rospy.Subscriber('dgps_heading', heading_ang, self.get_heading)

    def utm_pub(self):
        rospy.init_node('gps_pose', anonymous=False)
        pub = rospy.Publisher('odom_pose', coordinates, queue_size=10)
        pub.publish(self.X,self.Y,self.HEADING)
        rospy.sleep(0.01)

if __name__== '__main__':
    gps_pose_obj = gps_pose_node()
    print("[ INFO] Initialized GPS and Heading Node.")
    while not rospy.is_shutdown():
        gps_pose_obj.gps_sub()
        gps_pose_obj.mag_sub()
        gps_pose_obj.utm_pub()

	     # Using Scout Odometer
		# odom_sub()
		# odom_pub()

