#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy
import sys
from gps_nav.msg import can_pose, pose_xy
from gps_nav.srv import feedback_srv, feedback_srvResponse
from geometry_msgs.msg import Twist

# x_dest, y_dest, theta_dest = 0, 0, 0

class move_node(object):
	def __init__(self):
		self.TOP_LINEAR_SPEED = 0.15
		self.TOP_ANGULAR_SPEED = 0.15
		self.linear_spd = 0.11
		self.angular_spd = 0.11
		#self.flag_grt, self.flag_sml = False, False
		self.x_est, self.y_est, self.theta_est, self.theta_done, self.linear_done = 0, 0, 0, False, False

	def regulate(self, linear_spd, angular_spd):
		if self.linear_spd > self.TOP_LINEAR_SPEED:
			print("[ INFO] Fixing Linear Speed ...")
			self.linear_spd = self.TOP_LINEAR_SPEED
		if self.angular_spd > self.TOP_ANGULAR_SPEED:
			print("[ INFO] Fixing Angular Speed ...")
			self.angular_spd = self.TOP_ANGULAR_SPEED

		return self.linear_spd, self.angular_spd

	def to_2pi(self, angle):
		if angle < 0 and angle > -180:
			return 360-angle
		return angle

	def scout_ctrl_command(self):
		T = Twist()
		rospy.init_node('scout_ctrl', anonymous=False)
		pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		self.linear_spd, self.angular_spd = self.regulate(self.linear_spd, self.angular_spd)

		hyp = (self.x_est**2 + self.y_est**2)**0.5
		if (not self.theta_done and not self.linear_done) or self.theta_done and self.linear_done:
			if self.theta_done:
				T.linear.x = 0
				T.linear.y = 0
				T.linear.z = 0
				T.angular.z = 0

			elif self.to_2pi(self.theta_est) >= 180:
				T.linear.x = 0
				T.linear.y = 0
				T.linear.z = 0
				T.angular.z = -1*self.angular_spd

			elif self.to_2pi(self.theta_est) < 180:
				T.linear.x = 0
				T.linear.y = 0
				T.linear.z = 0
				T.angular.z = self.angular_spd

		elif self.theta_done and not self.linear_done:
			T.linear.x = self.linear_spd
			T.linear.y = 0
			T.linear.z = 0
			T.angular.z = 0

		else:
			pass

		pub.publish(T)

	def sub_ctrl_msg(self, req):
		self.x_est, self.y_est, self.theta_est, self.theta_done, self.linear_done = req.x, req.y, req.theta, req.theta_done, req.linear_done
		#print(self.x_est, self.y_est, self.theta_est, self.theta_done, self.linear_done)
		rcv = True
		if rcv:
			self.scout_ctrl_command()
			rcv = False
			return feedback_srvResponse(True)
		else:
			print("Did not work!")
			return feedback_srvResponse(False)

	def feedback_ctrl_msg(self):
		rospy.init_node('scout_ctrl', anonymous=False)
		fb_s = rospy.Service("feedback_srv", feedback_srv, self.sub_ctrl_msg)

if __name__ == '__main__':
	move_obj = move_node()
	move_obj.feedback_ctrl_msg()
	while not rospy.is_shutdown():
		move_obj.scout_ctrl_command()
