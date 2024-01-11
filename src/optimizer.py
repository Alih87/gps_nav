#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, flag
#from sbg_driver.msg import SbgGpsPos, SbgMag
from math import atan, atan2, pi

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
	def __init__(self):
		self.theta_done, self.linear_done = False, False
		self.curr_x, self.curr_y, self.curr_theta = 0, 0, 0
		self.dest_x, self.dest_y, self.dest_theta = 0, 0, 0
		self.x, self.y, self.theta = 0, 0, 0
		
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

	def update_done_flag(self, done=False):
		rospy.init_node('optimizer', anonymous=False)
		pub = rospy.Publisher('done_flag', flag, queue_size=10)
		pub.publish(done)
	
	def get_dest_state(self, data):
		self.dest_x, self.dest_y, self.dest_theta = data.x, data.y, data.theta

	def get_state(self, data):
		self.curr_x, self.curr_y, self.curr_theta = data.x, data.y, data.theta

	def get_dest_pose(self):
		rospy.init_node('optimizer', anonymous=False)
		rospy.Subscriber('final_pos', coordinates, self.get_dest_state)
		rospy.sleep(0.01)

	def get_curr_pose(self):
		rospy.init_node('optimizer', anonymous=False)
		rospy.Subscriber('odom_pose', coordinates, self.get_state)
		rospy.sleep(0.01)

	def to_go(self):
		rospy.init_node('optimizer', anonymous=False)
		pub = rospy.Publisher('feedback', pose_xy, queue_size=30)
		self.x = self.dest_x - self.curr_x
		self.y = self.dest_y - self.curr_y
		self.theta = (self.calculate_angle2(self.x, self.y) - self.curr_theta)
		if self.theta > 180:
			self.theta = self.theta - 360
		elif self.theta < -180:
			self.theta = 360 + self.theta

		'''
		Checks whether the current angle is within the 90 degree (at max) arc.
		'''
		if self.theta < -30 or self.theta > 30:
			self.theta_done = False
		else:
			self.theta_done = True

		'''
		Checks whether the current position is within 15 meters range (at max).
		'''
		if (self.x**2 + self.y**2)**0.5 < 0.5:
			self.linear_done = True
		else:
			self.linear_done = False

		'''
		If both angular and linear position is within the required range, complete the path and move to the next destination points.
		'''
		if self.linear_done and self.theta_done:
			self.theta_done, self.linear_done = False, False
			self.update_done_flag(True)
		else:
			self.update_done_flag()

		pub.publish(self.x, self.y, self.theta, self.theta_done, self.linear_done)
		rospy.sleep(0.01)

if __name__ == '__main__':
	optim_obj = optimizer_node()
	print("[INFO] Initialized Optimization Node.")
	while not rospy.is_shutdown():
		optim_obj.get_curr_pose()
		optim_obj.get_dest_pose()
		optim_obj.to_go()

	#get_xy_pose()
        #get_dest_xy_pose()
        #dist_to_go()
        # print("x : ", dest_x - curr_x)
        # print("y : ", dest_y - curr_y)
        # print("theta : ", dest_theta - curr_theta, "\n\n")
        # print("displacement : ", ((dest_x - curr_x)**2 + (dest_y - curr_y)**2)**0.5)
