#!/usr/bin/env python
import roslib, math
import tf.transformations; roslib.load_manifest('gps_nav')
import rospy, sys
from collections import deque
from gps_nav.msg import coordinates, pose_xy, latlon_gps, heading_ang
from gps_nav.srv import gps_pos_srv, utm_srv, gps_pos_srvResponse
#from sbg_driver.msg import SbgGpsPos, SbgMag
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, MagneticField
from geometry_msgs.msg import Vector3Stamped, TransformStamped
from sbg_driver.msg import SbgEkfEuler, SbgMag
from utm import from_latlon
import tf2_ros, tf_conversions, os, datetime
from math import atan, atan2, pi
from numpy import random, array, linalg, matmul, zeros, eye

class heading_KF(object):
	def __init__(self):
		pass

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
		self.buf = tf2_ros.Buffer()
		self.ls = tf2_ros.TransformListener(self.buf)
		self.t = TransformStamped()
		self.lat, self.long = 0, 0

		self.prioris, self.yaws, self.mags, self.posts = [], [], [], []

	def to_2pi(self, angle):
		if angle < 0:
			return 360+angle
		return angle
	
	def euler_from_quaternion(self, x, y, z, w):
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)

		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)

		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)

		return yaw_z * (180/math.pi)

	def lookup_trans(self):
		# try:
		# 	self.dest_l.waitForTransform("destination", "scout", rospy.Time(0), rospy.Duration(10))
		# 	self.trans, self.rot = self.dest_l.lookupTransform('destination', 'scout', rospy.Time(0))
		# 	self.x, self.y, self.theta = self.trans[0], self.trans[1], self.euler_from_quaternion(*self.rot)
		# 	self.theta = self.to_2pi(self.theta)
		# 	print(self.trans, self.rot)
		# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 	pass
		if self.buf.can_transform('utm', 'base_link', rospy.Time(0), rospy.Duration(2)):
			trans = self.buf.lookup_transform('utm', 'base_link', rospy.Time())
			self.X = trans.transform.translation.x
			self.Y = trans.transform.translation.y
			rx = trans.transform.rotation.x
			ry = trans.transform.rotation.y
			rz = trans.transform.rotation.z
			rw = trans.transform.rotation.w
			self.HEADING = self.euler_from_quaternion(*[rx,ry,rz,rw])
			# print(self.x, self.y, self.theta)
		else:
			rospy.logdebug("Can't Transform")


	def get_utm(self, data):
		self.lat, self.long = data.latitude, data.longitude
		self.X, self.Y, zo, ne = from_latlon(self.lat, self.long)
		#self.X, self.Y = self.X - self.CENTER[0], self.Y - self.CENTER[1]
		#self.X, self.Y = self.X, self.Y
		#ZONE = str(zo)+ne
		# self.posts.append(self.HEADING)
		# self.broadcast_transform()

	def get_utm_srv(self, req):
		if not self.scout_odom:
			lat, long = req.lat, req.lon
			self.X, self.Y, zo, ne = from_latlon(lat, long)
			if self.X or self.Y:
				self.posts.append(self.HEADING)

				return gps_pos_srvResponse(True)
			else:
				return gps_pos_srvResponse(False)
		else:
			rospy.init_node('gps_pose', anonymous=False)
			rospy.Subscriber('odom', Odometry, self.get_scout_odom)
			return gps_pos_srvResponse(True)

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
            # For chip IMU
			angle = atan2(data.magnetic_field.y, data.magnetic_field.x) * (180/pi)
			self.HEADING = angle

	def gps_sub(self):
		if not self.scout_odom:
			# rospy.init_node('gps_pose', anonymous=False)
			rospy.Subscriber('ublox/fix', NavSatFix, self.get_utm)
		else:
			# rospy.init_node('gps_pose', anonymous=False)
			rospy.Subscriber('odom', Odometry, self.get_scout_odom)

	def gps_sub_Service(self):
		# rospy.init_node('gps_pose', anonymous=False)
		gps_srv = rospy.Service('gps_pos_srv2', gps_pos_srv, self.get_utm_srv)

	def mag_sub(self):
		# rospy.init_node('gps_pose', anonymous=False)
		if (not self.is_scout) and (not self.imu_ros):
				rospy.Subscriber('um7_heading', heading_ang, self.get_heading)
		elif self.is_scout:
			rospy.Subscriber('odom', Odometry, self.get_heading)
		elif self.imu_ros:
			rospy.Subscriber('/imu/mag', MagneticField, self.get_heading)

	def utm_pub(self):
		# rospy.init_node('gps_pose', anonymous=False)
		pub = rospy.Publisher('odom_pose', coordinates, queue_size=1)
		pub.publish(self.X,self.Y,self.HEADING)
		#rospy.sleep(0.025)

	def utm_pub_srv(self):
		# rospy.init_node('gps_pose', anonymous=False)
		pub = rospy.ServiceProxy('utm_srv', utm_srv)
		resp = pub(self.X, self.Y, self.HEADING)
	
		if not resp.done:
			raise Exception("[INFO] False response from utm publishing Service.")
	
if __name__== '__main__':
	rospy.init_node('gps_pose', anonymous=False)
	rate = rospy.Rate(15)  #15 Hz
	gps_pose_obj = gps_pose_node(is_scout=False, scout_odom=False, imu_ros=True)
	home_dir = os.environ['HOME']
	dt = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M:%S")

	print("[ INFO] Initialized GPS and Heading Node.")
	# gps_pose_obj.gps_sub_Service()
	with open(home_dir+"/boat_data/cont/CONT_LOG_"+dt+".txt", 'w') as f:
		while not rospy.is_shutdown():
			# print("Current Position", gps_pose_obj.X, gps_pose_obj.Y, gps_pose_obj.HEADING)
			# gps_pose_obj.gps_sub()
			gps_pose_obj.lookup_trans()
			# gps_pose_obj.mag_sub()
			# gps_pose_obj.utm_pub()
			#gps_pose_obj.utm_pub_srv()
			# Using Scout Odometer
			# odom_sub()
			# odom_pub()
			f.write(str(gps_pose_obj.X)+","+str(gps_pose_obj.Y)+","+str(gps_pose_obj.HEADING)+"\n")
			rate.sleep()
			
		f.close()

