#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy
import sys
from gps_nav.msg import can_pose
from gps_nav.msg import pose_xy
from geometry_msgs.msg import Twist

# x_dest, y_dest, theta_dest = 0, 0, 0

class move_node(object):
	def __init__(self):
		self.TOP_LINEAR_SPEED = 10
		self.TOP_ANGULAR_SPEED = 10
		self.linear_spd = 10
		self.angular_spd = 10
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

	def scout_ctrl_command(self):
		rospy.init_node('scout_ctrl', anonymous=False)
		pub = rospy.Publisher('cmd_vel', can_pose, queue_size=30)

		self.linear_spd, self.angular_spd = self.regulate(self.linear_spd, self.angular_spd)

		hyp = (self.x_est**2 + self.y_est**2)**0.5
		if not self.theta_done and not self.linear_done:
			if self.theta_est == 0
				pass

			elif self.theta_est/abs(self.theta_est) > 0:
				pass

			elif self.theta_est/abs(self.theta_est) < 0:
				pass

		elif self.theta_done and not self.linear_done:
			pass

		else:
			pass

		pub.publish(rt, lt)

	def sub_ctrl_msg(self, data):
		global x_est, y_est, theta_est, theta_done, linear_done
		self.x_est, self.y_est, self.theta_est, self.theta_done, self.linear_done = data.x, data.y, data.theta, data.theta_done, data.linear_done


	def feedback_ctrl_msg(self):
		rospy.init_node('scout_ctrl', anonymous=False)
		rospy.Subscriber("feedback", pose_xy, self.sub_ctrl_msg)

if __name__ == '__main__':
	move_obj = move_node()
	while not rospy.is_shutdown():
		move_obj.scout_ctrl_command()
		move_obj.feedback_ctrl_msg()
