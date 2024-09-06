#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys, datetime
from gps_nav.srv import flag_srv, feedback_srv, logging_srv, final_pos_srv, utm_srv, final_pos_srvResponse, utm_srvResponse, cont_log_srv, cont_log_srvResponse
from gps_nav.msg import coordinates, pose_xy, flag
from math import atan, atan2, pi
import tf, math

class optimizer_node():
	def __init__(self, collect_data=False):
		self.theta_done, self.linear_done = False, False
		self.curr_x, self.curr_y, self.curr_theta = 0, 0, 0
		self.dest_x, self.dest_y, self.dest_theta = 0, 0, 0
		self.x, self.y, self.theta = 0, 0, 0
		self.collect_data = collect_data
		self.last_dest, self.strikes = 0, 0
		self.trans, self.rot = 0, 0

		self.dest_l = tf.TransformListener()

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

	def utm_map_listener(self):
		try:
			self.trans, self.rot = self.dest_l.lookupTransform('map', 'destination', rospy.Time(0))
			x, y, theta = self.trans[0], self.trans[1], self.euler_from_quaternion(*self.rot)
			self.dest_x, self.dest_y, self.dest_theta = x, y, theta
			# print("Next Destination", self.dest_x, self.dest_y, self.dest_theta)
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			# print("Next Destination", self.dest_x, self.dest_y)
			pass

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
		self.curr_x, self.curr_y, self.curr_theta = req.x, req.y, -1*req.theta
		# if self.curr_theta < -180:
		# 	self.curr_theta += 360
		# print(self.curr_x, self.curr_y, self.curr_theta)
		rcv = True
		if rcv:
			rcv = True
			return utm_srvResponse(True)
		else:
			return utm_srvResponse(False)

	def get_dest_pose(self):
		rospy.Service('final_pos_srv', final_pos_srv, self.get_dest_state)

	def cont_log_cb(self, req):
		if req.r:
			return cont_log_srvResponse(self.curr_x, self.curr_y, self.curr_theta)

	def get_curr_pose(self):
		rospy.Service('utm_srv', utm_srv, self.get_state)

	def contLoggingServer(self):
		rospy.Service('cont_log', cont_log_srv, self.cont_log_cb)

	def LoggingSrvProxy(self):
		rospy.wait_for_service('logg_srv')
		dt = str(datetime.datetime.now()).split('.')[0]
		sec, mn_sec, hr_sec = int(dt[-2:]), int(dt[-5:-3])*60, int(dt[-8:-6])*3600
		pub = rospy.ServiceProxy('logg_srv', logging_srv)
		resp  = pub(self.curr_x, self.curr_y, self.theta, int(sec+mn_sec+hr_sec))
		if not resp.done:
			raise Exception("[INFO] False response from Logging Service.")

	def to_go(self):
		rospy.wait_for_service('feedback_srv')
		pub = rospy.ServiceProxy('feedback_srv', feedback_srv)

		self.x = self.dest_x - self.curr_x
		self.y = self.dest_y - self.curr_y
		tgt_theta = self.calculate_angle2(self.x, self.y)
		# if tgt_theta > 180:
		# 	tgt_theta =- 360
		# if tgt_theta < -180:
		# 	tgt_theta += 360
		self.theta = self.curr_theta - tgt_theta + 180
		if self.theta >= 180:
			self.theta = self.theta - 360
		elif self.theta < -180:
			self.theta = self.theta + 360
		# print(self.curr_theta, tgt_theta, self.theta)
		# print(self.curr_x, self.curr_y)
		print(self.x, self.y, self.theta, self.curr_theta)
		print(self.curr_x, self.curr_y)
		print(self.dest_x, self.dest_y)

		'''
		Checks whether the current angle is within the 3 degree (at max) arc.
		'''
		if (self.theta < -1.0 or self.theta > 1.0) and not self.theta_done:
			self.theta_done = False
		else:
			self.theta_done = True
			resp = pub(self.x, self.y, self.theta, self.theta_done, self.linear_done)
			if not resp.done:
				raise Exception("[INFO] False response from Optimizer Service.")

		'''
		Checks whether the current position is within 55 centimeters radius (at max).
		'''
		if ((self.x**2 + self.y**2)**0.5 > 0.55) and self.theta_done:
			self.linear_done = False

		elif self.theta_done and not self.linear_done:
			self.linear_done = True
			resp = pub(self.x, self.y, self.theta, self.theta_done, self.linear_done)
			if not resp.done:
				raise Exception("[INFO] False response from Optimizer Service.")

		'''
		If both angular and linear position is within the required range, complete the path and move to the next destination points.
		'''
		if self.linear_done and self.theta_done:
			self.theta_done, self.linear_done = False, False
			if not self.collect_data:
				self.LoggingSrvProxy()
				self.update_flag_srv()
				resp = pub(self.x, self.y, self.theta, self.theta_done, self.linear_done)
				if not resp.done:
					raise Exception("[INFO] False response from Optimizer Service.")
			self.theta_done, self.linear_done = False, False

		elif not self.linear_done and not self.theta_done:
			self.theta_done, self.linear_done = False, False
			resp = pub(self.x, self.y, self.theta, self.theta_done, self.linear_done)
			if not resp.done:
				raise Exception("[INFO] False response from Optimizer Service.")
		rospy.sleep(0.005)

if __name__ == '__main__':
	rospy.init_node('optimizer', anonymous=False)
	rate = rospy.Rate(30)
	run_once = True	# Make 'run_once' True when collecting data
	optim_obj = optimizer_node(collect_data=False)
	print("[INFO] Initialized Optimization Node.")
	optim_obj.get_curr_pose()
	# optim_obj.contLoggingServer()
	while not rospy.is_shutdown():
		optim_obj.utm_map_listener()
		optim_obj.to_go()
		if not run_once:
			optim_obj.LoggingSrvProxy()
			run_once = True
		elif run_once and optim_obj.collect_data:
			optim_obj.LoggingSrvProxy()
		rate.sleep()
	