#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from collections import deque
from gps_nav.msg import coordinates, pose_xy, latlon_gps, heading_ang
from gps_nav.srv import gps_pos_srv, utm_srv, gps_pos_srvResponse
#from sbg_driver.msg import SbgGpsPos, SbgMag
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from sbg_driver.msg import SbgEkfEuler, SbgMag
from utm import from_latlon
from math import atan, atan2, pi
from numpy import random, array, linalg, matmul, zeros, eye

class heading_KF(object):
	def __init__(self):
		self.priori_est, self.post_est, self.P = array(([[0]]),ndmin=2), array(([[0]]),ndmin=2), array(([[1]]))
		self.mag_measure = 0
		self.yaw_measure = 0
		self.declination = -8.51667
		self.MAX_LEN = 1
		
		self.moving_avg = deque(maxlen=self.MAX_LEN)

		self.initial_heading, self.offet = 0, 0
		self.theta_scout, self.theta_mag, self.theta_yaw = 0, 0, 0
		self.prev_theta_scout, self.prev_theta_mag, self.delta_theta = 0, 0, 0
		self.A, self.B = array(([[1]]), ndmin=2), array(([[1]]), ndmin=2)

	def get_mag_theta(self, data):
		x = data.mag.x
		y = data.mag.y
		theta = 1*atan2(y,x)*(180/pi)
		#theta = ((2*pi + theta)*(theta<0) + theta*(theta>0))*(180/pi) + self.declination
		#if abs(theta - self.prev_theta_mag) >= 190:
		#	theta = self.prev_theta_mag
		#	theta = (theta + 180) % 360 - 180
		#else:
		#	self.prev_theta_mag = theta
		self.moving_avg.append(theta)
		self.theta_mag = sum(self.moving_avg)/self.MAX_LEN
	
	def get_yaw_theta(self, data):
		self.theta_yaw = -1*data.angle.z*(180/pi)
		#self.theta_yaw = ((2*pi+self.theta_yaw)*(self.theta_yaw < 0) + self.theta_yaw*(self.theta_yaw > 0))*(180/pi)

	def get_scout_theta(self, data):
		z = data.pose.pose.orientation.z
		w = data.pose.pose.orientation.w
		self.theta_scout = atan2(2.0*(w*z), 1.0-2.0*(z*z))*(180/pi)
		#self.theta_scout = ((2*pi+self.theta_scout)*(self.theta_scout < 0) + self.theta_scout*(self.theta_scout > 0))*(180/pi)
		self.delta_theta = self.theta_scout-self.prev_theta_scout
		self.prev_theta_scout = self.theta_scout

	def get_initial_heading(self):
		rospy.init_node('gps_pose', anonymous=False)
		rospy.Subscriber('odom', Odometry, self.get_scout_theta)
		rospy.Subscriber('/sbg/mag', SbgMag, self.get_mag_theta)
		rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.get_yaw_theta)
		self.initial_heading = self.theta_mag
		self.prev_theta_mag = self.initial_heading
		self.moving_avg.extend([self.initial_heading]*self.MAX_LEN)
		self.offset = self.theta_mag - self.theta_yaw
		self.prev_theta_scout = self.theta_scout
		
		rospy.sleep(0.001)

	def get_priori_est(self):
		rospy.init_node('gps_pose', anonymous=False)
		rospy.Subscriber('odom', Odometry, self.get_scout_theta)
		self.priori_est = matmul(self.A, array(([[self.theta_scout + self.initial_heading]]), ndmin=2)) + matmul(self.B, array(([[self.delta_theta]]), ndmin=2))
		self.priori_est = (self.priori_est + 180) % 360 - 180
	
	def get_measurements(self):
		rospy.init_node('gps_pose', anonymous=False)
		rospy.Subscriber('/sbg/mag', SbgMag, self.get_mag_theta)
		rospy.Subscriber('/sbg/ekf_euler', SbgEkfEuler, self.get_yaw_theta)
		self.mag_measure, self.yaw_measure = self.theta_mag, self.theta_yaw + self.offset
		self.mag_measure, self.yaw_measure = (self.mag_measure + 180) % 360 - 180, (self.yaw_measure + 180) % 360 - 180

	def get_posteriori_est(self):
		Q = random.normal(0, 0.7, 1)[0]
		C = array(([[1],[1]]), ndmin=2)
		R = array(([[random.normal(0, 1.959, 1)[0], 0],
			    [0, 0.001]]))
		last_K_ = array(([[0.5, 0.5]]), ndmin=2)

		# Get priori estimate and predicted error covariance
		self.get_priori_est()
		P_ = matmul(self.A, matmul(self.P, self.A.T)) + Q

		# Update using measurements
		K_ = matmul(matmul(P_, C.T), linalg.inv(matmul(C, matmul(P_, C.T)) + R))
		self.get_measurements()
		y = array(([[self.mag_measure],[self.yaw_measure]]), ndmin=2)
		if self.priori_est - matmul(K_,(y - matmul(C, self.priori_est))) > 185 or self.priori_est - matmul(K_,(y - matmul(C, self.priori_est))) < -185:
			self.post_est += self.delta_theta
		else:	
			self.post_est = self.priori_est - matmul(K_,(y - matmul(C, self.priori_est)))
		self.post_est = (self.post_est + 180) % 360 - 180
		self.P = matmul((array([[1]],ndmin=2) - matmul(K_, C)), P_)
		#self.priori_est = self.post_est


# HEADING = deque(maxlen=5)
class gps_pose_node(object):
    def __init__(self, kf_object, is_scout=False, scout_odom=False, imu_ros=False):
        #self.CENTER = (388731.70, 3974424.49)
        self.ZONE = ''
        self.X = 0
        self.Y = 0
        self.HEADING = 0
	self.is_scout = is_scout
	self.imu_ros = imu_ros
	self.scout_odom = scout_odom
	self.ang_count = 0
	self.kf_obj = kf_object

	self.prioris, self.yaws, self.mags, self.posts = [], [], [], []
	for _ in range(1500):
		self.kf_obj.get_initial_heading()

    def get_utm(self, data):
        lat, long = data.lat, data.lon
        self.X, self.Y, zo, ne = from_latlon(lat, long)
        #self.X, self.Y = self.X - self.CENTER[0], self.Y - self.CENTER[1]
        #self.X, self.Y = self.X, self.Y
        #ZONE = str(zo)+ne

    def get_utm_srv(self, req):
	if not self.scout_odom:
		lat, long = req.lat, req.lon
		self.X, self.Y, zo, ne = from_latlon(lat, long)
		if self.X or self.Y:
			self.kf_obj.get_posteriori_est()
			self.yaws.append(self.kf_obj.yaw_measure)
			self.mags.append(self.kf_obj.mag_measure)			
			self.prioris.append(self.kf_obj.priori_est[0][0])
			print(self.kf_obj.mag_measure)
			print(self.kf_obj.theta_yaw)
			print(self.kf_obj.theta_scout)
			self.HEADING = self.kf_obj.post_est[0][0]
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
		angle = data.vector.z*(180/pi)
		
		# For sbg
		#angle = data.angle.z*(180/pi)
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
		rospy.Subscriber('/imu/mag', Vector3Stamped, self.get_heading)

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
    kf_obj = heading_KF()
    gps_pose_obj = gps_pose_node(kf_obj, is_scout=False, scout_odom=False, imu_ros=True)
    print("[ INFO] Initialized GPS and Heading Node.")
    gps_pose_obj.gps_sub_Service()
    while not rospy.is_shutdown():
	print("Current Position", gps_pose_obj.X, gps_pose_obj.Y, gps_pose_obj.HEADING)
        #gps_pose_obj.gps_sub()
        #gps_pose_obj.mag_sub()
        #gps_pose_obj.utm_pub()
	gps_pose_obj.utm_pub_srv()
	     # Using Scout Odometer
		# odom_sub()
		# odom_pub()
	with open("/home/scout/boat_data/sbg_kf_angles.txt", 'w') as f:
		for i,j,w,k in zip(gps_pose_obj.yaws, gps_pose_obj.mags, gps_pose_obj.prioris, gps_pose_obj.posts):
			f.write(str(i)+","+str(j)+","+str(w)+","+str(k)+"\n")
		f.close()

