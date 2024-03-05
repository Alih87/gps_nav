#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, latlon_gps, heading_ang
from gps_nav.srv import gps_pos_srv, utm_srv, gps_pos_srvResponse
#from sbg_driver.msg import SbgGpsPos, SbgMag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from sbg_driver.msg import SbgEkfEuler, SbgMag
from utm import from_latlon
from math import atan, atan2, pi

# HEADING = deque(maxlen=5)
class gps_pose_node(object):
    def __init__(self, is_scout=False, scout_odom=False, imu_ros=False):
        #self.CENTER = (388731.70, 3974424.49)
        self.ZONE = ''
        self.X = 0
        self.Y = 0
        self.HEADING = 0
	self.is_scout = is_scout
	self.imu_ros = imu_ros
	self.scout_odom = scout_odom
	self.ang_count = 0

    def get_utm(self, data):
        lat, long = data.lat, data.lon
        self.X, self.Y, zo, ne = from_latlon(lat, long)
        #self.X, self.Y = self.X - self.CENTER[0], self.Y - self.CENTER[1]
        #self.X, self.Y = self.X, self.Y
        #ZONE = str(zo)+ne

    def get_utm_srv(self, req):
	lat, long = req.lat, req.lon
        self.X, self.Y, zo, ne = from_latlon(lat, long)
	if self.X or self.Y:
		return gps_pos_srvResponse(True)
	else:
		return gps_pos_srvResponse(False)

    def get_scout_odom(self, data):
	x = data.pose.pose.position.x
	y = data.pose.pose.position.y
	self.X, self.Y = x, y

    def get_heading(self, data):
	if not self.is_scout and not self.imu_ros:
		angle = data.angle
		self.HEADING = angle

	elif self.is_scout:
		z = data.pose.pose.orientation.z
		w = data.pose.pose.orientation.w
		angle = atan2(2.0*(w*z), 1.0 - 2.0*(z*z))*(180/pi)
		#rospy.sleep(0.5)
		#while self.ang_count != 16:
		#	angle += angle
		#	self.ang_count += 1
		self.HEADING = angle

	elif self.imu_ros:
		# For UM7 IMU pose
		#angle = data.vector.z*(180/pi)

		# For UM7 IMU mag
		x = data.vector.x
		y = data.vector.y
		angle = atan2(y,x)*(180/pi)

		# For sbg pose
		#angle = data.angle.z*(180/pi)

		# For sbg mag
		#x = data.mag.x
		#y = data.mag.y
		#angle = atan2(y,x)*(180/pi)

		self.HEADING = angle

    def gps_sub(self):
	if not self.scout_odom:
		rospy.init_node('gps_pose', anonymous=False)
		rospy.Subscriber('gps_pos2', latlon_gps, self.get_utm)
	else:
		rospy.init_node('gps_pose', anonymous=False)
		rospy.Subscriber('odom', Odometry, self.get_scout_odom)

    def gps_sub_Service(self):
	rospy.init_node('gps_pose', anonymous=False)
	gps_srv = rospy.Service('gps_pos_srv', gps_pos_srv, self.get_utm_srv)

    def mag_sub(self):
        rospy.init_node('gps_pose', anonymous=False)
	if (not self.is_scout) and (not self.imu_ros):
        	rospy.Subscriber('um7_heading', heading_ang, self.get_heading)
	elif self.is_scout:
		rospy.Subscriber('odom', Odometry, self.get_heading)
	elif self.imu_ros:
		rospy.Subscriber('/imu/rpy', Vector3Stamped, self.get_heading)

    def utm_pub(self):
        rospy.init_node('gps_pose', anonymous=False)
        pub = rospy.Publisher('odom_pose', coordinates, queue_size=1)
        pub.publish(self.X,self.Y,self.HEADING)
	rospy.sleep(0.025)

    def utm_pub_srv(self):
	rospy.init_node('gps_pose', anonymous=False)
	pub = rospy.ServiceProxy('utm_srv', utm_srv)
	resp = pub(self.X, self.Y, self.HEADING)
	
	if not resp.done:
		raise Exception("[INFO] False response from utm publishing Service.")

if __name__== '__main__':
    gps_pose_obj = gps_pose_node(is_scout=False, scout_odom=False, imu_ros=True)
    print("[ INFO] Initialized GPS and Heading Node.")
    while not rospy.is_shutdown():
        gps_pose_obj.mag_sub()
	print("Current Angle", gps_pose_obj.HEADING)

