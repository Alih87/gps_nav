#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, flag
#from sbg_driver.msg import SbgGpsPos, SbgMag
from math import atan, atan2, pi


global theta_done, linear_done
theta_done, linear_done = False, False
curr_x, curr_y, curr_theta = 0, 0, 0
dest_x, dest_y, dest_theta = 0, 0, 0
x, y, theta = 0, 0, 0

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

def calculate_angle(y, x):
	if x > 0:
		return atan(y/x)*(180/pi)
	if x==0 and y > 0:
		return (pi/2)*(180/pi)
	if x==0 and y < 0:
		return (-pi/2)*(180/pi)
	if x < 0 and y >= 0:
		return (atan(y/x) + pi)*(180/pi)
	if x < 0 and y < 0:
		return (atan(y/x) - pi)*(180/pi)

def calculate_angle2(y, x):
		return atan2(y,x)*(180/pi)

def make_done_false():
	global theta_done, linear_done
	theta_done, linear_done = False, False

def update_done_flag(done=False):
	rospy.init_node('optimizer', anonymous=False)
	pub = rospy.Publisher('done_flag', flag, queue_size=10)
	pub.publish(done)

def get_dest_pose():
    rospy.init_node('optimizer', anonymous=False)
    rospy.Subscriber('final_pos', coordinates, get_dest_state)
    rospy.sleep(0.01)

def get_curr_pose():
    rospy.init_node('optimizer', anonymous=False)
    rospy.Subscriber('odom_pose', coordinates, get_state)
    rospy.sleep(0.01)

def to_go():
    rospy.init_node('optimizer', anonymous=False)
    pub = rospy.Publisher('feedback', pose_xy, queue_size=30)
    global x, y, theta, theta_done, linear_done
    x = dest_x - curr_x
    y = dest_y - curr_y
    theta = calculate_angle2(y, x) - curr_theta

    if abs(theta) < 3:
	theta_done = True
    if abs((x**2 + y**2)**0.5) < 1:
	linear_done = True
	
    if linear_done and theta_done:
	theta_done, linear_done = False, False
	update_done_flag(True)
    else:
	update_done_flag()
	
    
    pub.publish(x, y, theta, theta_done, linear_done)
    rospy.sleep(0.01)

if __name__ == '__main__':
    print("[INFO] Initialized Optimization Node.")
    while not rospy.is_shutdown():
        get_curr_pose()
        get_dest_pose()
        to_go()

	#get_xy_pose()
        #get_dest_xy_pose()
        #dist_to_go()
        # print("x : ", dest_x - curr_x)
        # print("y : ", dest_y - curr_y)
        # print("theta : ", dest_theta - curr_theta, "\n\n")
        # print("displacement : ", ((dest_x - curr_x)**2 + (dest_y - curr_y)**2)**0.5)
