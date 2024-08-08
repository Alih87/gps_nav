#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys, datetime
from gps_nav.srv import flag_srv, feedback_srv, logging_srv, final_pos_srv, utm_srv, final_pos_srvResponse, utm_srvResponse, cont_log_srv, cont_log_srvResponse
from gps_nav.msg import coordinates, pose_xy, flag
#from sbg_driver.msg import SbgGpsPos, SbgMag
from math import atan, atan2, pi
from geometry_msgs.msg import Twist
import math
import tf2_ros, os

############### USING SCOUT ODOMETER #####################

def get_theta(q):
    siny_cosp = 2*(q.w*q.z + q.x*q.z)


    cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)

    return atan2(siny_cosp, cosy_cosp)*(180/pi)

def get_dest_state(data):
    global dest_x, dest_y, dest_theta
    dest_x, dest_y, dest_theta = data.x, data.y, data.theta

def get_state(data):
    global curr_x, curr_y, curr_theta
    curr_x, curr_y, curr_theta = data.x, data.y, data.theta

def get_dest_xy_pose():
    rospy.init_node('optimizer', anonymous=False)
    rospy.Subscriber('final_pos', coordinates, get_dest_state)
    rospy.sleep(0.01)

def get_xy_pose():
    rospy.init_node('optimizer', anonymous=False)
    rospy.Subscriber('odom_pose', coordinates, get_state)
    rospy.sleep(0.01)

def dist_to_go():

    rospy.init_node('optimizer', anonymous=False)
    pub = rospy.Publisher('feedback', pose_xy, queue_size=30)
    global x, y, theta
    x = dest_x - curr_x
    y = dest_y - curr_y
    theta = atan2(y, x)*(180/pi) - curr_theta
    
    pub.publish(x, y, theta, theta_done, linear_done)
    rospy.sleep(0.01)

##########################################################
class optimizer_node():
	def __init__(self, collect_data=False):
		self.theta_done, self.linear_done = False, False
		self.curr_x, self.curr_y, self.curr_theta = 0, 0, 0
		self.dest_x, self.dest_y, self.dest_theta = 0, 0, 0
		self.x, self.y, self.theta = 0, 0, 0
		self.collect_data = collect_data
		self.last_dest, self.strikes = 0, 0
		self.trans, self.rot = 0, 0
		self.tf_buffer = tf2_ros.Buffer()
		self.listener = tf2_ros.TransformListener(self.tf_buffer)

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

	def dest_listener(self):
		# try:
		# 	self.dest_l.waitForTransform("destination", "scout", rospy.Time(0), rospy.Duration(10))
		# 	self.trans, self.rot = self.dest_l.lookupTransform('destination', 'scout', rospy.Time(0))
		# 	self.x, self.y, self.theta = self.trans[0], self.trans[1], self.euler_from_quaternion(*self.rot)
		# 	self.theta = self.to_2pi(self.theta)
		# 	print(self.trans, self.rot)
		# except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
		# 	pass
		if self.tf_buffer.can_transform('destination', 'base_link', rospy.Time(0), rospy.Duration(2)):
			trans = self.tf_buffer.lookup_transform('destination', 'base_link', rospy.Time())
			self.x = trans.transform.translation.x
			self.y = trans.transform.translation.y
			rx = trans.transform.rotation.x
			ry = trans.transform.rotation.y
			rz = trans.transform.rotation.z
			rw = trans.transform.rotation.w
			self.theta = self.euler_from_quaternion(*[rx,ry,rz,rw])
			# print(self.x, self.y, self.theta)
		else:
			rospy.logdebug("Can't Transform")

	def calculate_angle(self, y, x):
		if x > 0:
			return atan(y/x)*(180/pi)
		if x == 0 and y > 0:
			return (pi/2)*(180/pi)
		if x == 0 and y < 0:
			return (-pi/2)*(180/pi)
		if x < 0 and y >= 0:
			return (atan(y/x) + pi)*(180/pi)
		if x < 0 and y < 0:
			return (atan(y/x) - pi)*(180/pi)

	def calculate_angle2(self, x, y):
		return atan2(y, x)*(180/pi)

	def make_done_false(self):
		self.theta_done, self.linear_done = False, False

	#def update_done_flag(self, done=False):
	#	rospy.init_node('optimizer', anonymous=False)
	#	pub = rospy.Publisher('done_flag', flag, queue_size=1)
	#	pub.publish(done)

	def update_flag_srv(self):
		rospy.wait_for_service('done_flag_srv')
		wps = rospy.ServiceProxy('done_flag_srv', flag_srv)
		try:
			srv_resp = wps(True)
			resp = srv_resp.fb
			if not resp:
				raise Exception("[INFO] False response from update flag Service.")
		except rospy.ServiceException as exc:
			print("[INFO] Flag Update Service did not process request: " + str(exc))

	def get_dest_state(self, req):
		self.dest_x, self.dest_y, self.dest_theta = req.x, req.y, req.theta
		rcv = self.dest_x == 0 and self.dest_y == 0
		if rcv:
			rcv = False
			return final_pos_srvResponse(True)
		elif (self.dest_x**2 + self.dest_y**2)**0.5 > 0.25:
			return final_pos_srvResponse(True)
		else:
			print("Did not work!")
			return final_pos_srvResponse(False)

	def get_state(self, req):
		self.curr_x, self.curr_y, self.curr_theta = req.x, req.y, req.theta
		rcv = True
		if rcv:
			rcv = True
			return utm_srvResponse(True)
		else:
			return utm_srvResponse(False)

	def get_dest_pose(self):
		# rospy.init_node('optimizer', anonymous=False)
		rospy.Service('final_pos_srv', final_pos_srv, self.get_dest_state)
		#rospy.sleep(0.01)

	def cont_log_cb(self, req):
		if req.r:
			return cont_log_srvResponse(self.curr_x, self.curr_y, self.curr_theta)

	def get_curr_pose(self):
		# rospy.init_node('optimizer', anonymous=False)
		rospy.Service('utm_srv', utm_srv, self.get_state)
		#rospy.sleep(0.01)
	
	def contLoggingServer(self):
		# rospy.init_node('optimizer', anonymous=False)
		rospy.Service('cont_log', cont_log_srv, self.cont_log_cb)

	def LoggingSrvProxy(self):
		# rospy.init_node('optimizer', anonymous=False)
		rospy.wait_for_service('logg_srv')
		dt = str(datetime.datetime.now()).split('.')[0]
		sec, mn_sec, hr_sec = int(dt[-2:]), int(dt[-5:-3])*60, int(dt[-8:-6])*3600
		pub = rospy.ServiceProxy('logg_srv', logging_srv)
		resp  = pub(self.curr_x, self.curr_y, self.theta, int(sec+mn_sec+hr_sec))
		if not resp.done:
			raise Exception("[INFO] False response from Logging Service.")
		
	def to_2pi(self, angle):
		if angle < 0:
			return 360+angle
		return angle

	def to_go(self):
		T = Twist()
		ang_range = 180
		lin_range = 2.5
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		hyp = (self.x**2 + self.y**2)**0.5
		if not self.theta_done or not self.linear_done:
			try:
				sign = (self.theta/abs(self.theta))
			except ZeroDivisionError:
				sign = 1
			ang_spd = (1 - abs(self.theta / ang_range)) * sign
			lin_spd = 1 - abs(hyp / lin_range)
			T.angular.z = ang_spd
			T.linear.x = lin_spd
		elif self.theta > -1.5 and self.theta < 1.5:
			self.theta_done = True
		elif hyp <= 0.50:
			self.linear_done = True
		elif self.theta_done and self.linear_done:
			self.theta_done, self.linear_done = False, False
			if not self.collect_data:
				self.LoggingSrvProxy()
				self.update_flag_srv()
			self.theta_done, self.linear_done = False, False
		# print(self.x, self.y, self.theta)
		pub.publish(T)
			
if __name__ == '__main__':
	rospy.init_node('optimizer', anonymous=False)
	rate = rospy.Rate(10)
	run_once = True	# Make 'run_once' True when collecting data
	optim_obj = optimizer_node(collect_data=False)
	print("[INFO] Initialized Optimization Node.")
	# optim_obj.get_dest_pose()
	# optim_obj.get_curr_pose()
	optim_obj.contLoggingServer()
	while not rospy.is_shutdown():
		optim_obj.dest_listener()
		optim_obj.to_go()
		if not run_once:
			optim_obj.LoggingSrvProxy()
			run_once = True
		elif run_once and optim_obj.collect_data:
			optim_obj.LoggingSrvProxy()
		# print("Next Destination", optim_obj.dest_x, optim_obj.dest_y)
		
