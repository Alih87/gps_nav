#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy
import sys
from gps_nav.msg import can_pose
from gps_nav.msg import pose_xy

TOP_LINEAR_SPEED = 10
TOP_ANGULAR_SPEED = 10
linear_spd = 10
angular_spd = 10
flag_grt, flag_sml = False, False

x_est, y_est, theta_est, theta_done, linear_done = 0, 0, 0, False, False
# x_dest, y_dest, theta_dest = 0, 0, 0

def regulate(linear_spd, angular_spd):
    global TOP_LINEAR_SPEED, TOP_ANGULAR_SPEED
    if linear_spd > TOP_LINEAR_SPEED:
        print("[ INFO] Fixing Linear Speed ...")
        linear_spd = TOP_LINEAR_SPEED
    if angular_spd > TOP_ANGULAR_SPEED:
        print("[ INFO] Fixing Angular Speed ...")
        angular_spd = TOP_ANGULAR_SPEED

    return linear_spd, angular_spd

def pub_ctrl_command():
    rospy.init_node('boat_ctrl', anonymous=False)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=30)

    global linear_spd, angular_spd

    linear_spd, angular_spd = regulate(linear_spd, angular_spd)

    hyp = (x_est**2 + y_est**2)**0.5
    if int(theta_est) is not 0 and hyp > 0.1:
        rt, lt = angular_spd, -angular_spd

    elif theta_est > 0 or theta_est < 0 or hyp > 0.1:
        rt, lt = linear_spd, linear_spd 
    
    else:
        rt, lt = 0, 0

    pub.publish(rt, lt)

def boat_ctrl_command():
	rospy.init_node('boat_ctrl', anonymous=False)
	pub = rospy.Publisher('cmd_vel', can_pose, queue_size=30)

	#hyp = (x_est**2 + y_est**2)**0.5
	global linear_spd, angular_spd, flag_grt, flag_sml

	linear_spd, angular_spd = regulate(linear_spd, angular_spd)

	hyp = (x_est**2 + y_est**2)**0.5
	if not theta_done and not linear_done:
		if theta_est == 0:
			rt, lt = 0, 0
			pass

		elif theta_est/abs(theta_est) > 0:
			if not flag_grt:
				for _ in range(4):
					rt, lt = 0, 0
				flag_grt = True
				flag_sml = False
				rospy.sleep(1)
			rt, lt = angular_spd, -angular_spd

		elif theta_est/abs(theta_est) < 0:
			if not flag_sml:
				for _ in range(4):
					rt, lt = 0, 0
				flag_grt = False
				flag_sml = True
				rospy.sleep(1)
			rt, lt = -angular_spd, angular_spd

	elif theta_done and not linear_done:
		rt, lt = linear_spd, linear_spd

	else:
		rt, lt = 0, 0

	pub.publish(rt, lt)

def sub_ctrl_msg(data):
    global x_est, y_est, theta_est, theta_done, linear_done
    x_est, y_est, theta_est, theta_done, linear_done = data.x, data.y, data.theta, data.theta_done, data.linear_done


def feedback_ctrl_msg():
    rospy.init_node('boat_ctrl', anonymous=False)
    rospy.Subscriber("feedback", pose_xy, sub_ctrl_msg)

if __name__ == '__main__':
	while not rospy.is_shutdown():
		boat_ctrl_command()
		feedback_ctrl_msg()
