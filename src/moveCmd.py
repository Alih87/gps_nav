#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy
import sys
from geometry_msgs.msg import Twist
from gps_nav.msg import pose_xy

TOP_LINEAR_SPEED = 0.1
TOP_ANGULAR_SPEED = 0.12
linear_spd = 0.1
angular_spd = 0.12

x_est, y_est, theta_est = 0, 0, 0
# x_dest, y_dest, theta_dest = 0, 0, 0

def regulate(linear_spd, angular_spd):
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

    # linear_spd, angular_spd = regulate(0.1, 0.12)

    twist = Twist()
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0

    # twist.linear.x = linear_spd
    # twist.angular.z = 0

    hyp = (x_est**2 + y_est**2)**0.5
    if int(theta_est) is not 0 and hyp > 0.1:
        twist.linear.x = 0
        twist.angular.z = (theta_est/abs(theta_est))*TOP_ANGULAR_SPEED

    elif theta_est > 0 or hyp > 0.1:
        twist.linear.x = TOP_LINEAR_SPEED
        twist.angular.z = 0
    
    else:
        twist.linear.x = 0
        twist.angular.z = 0

    pub.publish(twist)

def boat_ctrl_command():
	 

def sub_ctrl_msg(data):
    global x_est, y_est, theta_est
    x_est, y_est, theta_est = data.x, data.y, data.theta

# def sub_dest_msg(data):
#     global x_dest, y_dest, theta_dest
#     x_dest, y_dest, theta_dest = data.x, data.y, data.theta

def feedback_ctrl_msg():
    rospy.init_node('boat_ctrl', anonymous=False)
    rospy.Subscriber("feedback", pose_xy, sub_ctrl_msg)

if __name__ == '__main__':
    while not rospy.is_shutdown():
        feedback_ctrl_msg()
        pub_ctrl_command()
