#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy
from sbg_driver.msg import SbgGpsPos, SbgMag
from math import atan, atan2, pi

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
    rospy.Subscriber('final_pos', pose_xy, get_dest_state)
    rospy.sleep(0.1)

def get_xy_pose():
    rospy.init_node('optimizer', anonymous=False)
    rospy.Subscriber('odom_pose', pose_xy, get_state)
    rospy.sleep(0.1)

def dist_to_go():
    rospy.init_node('optimizer', anonymous=False)
    pub = rospy.Publisher('feedback', pose_xy, queue_size=30)
    global x, y, theta
    x = dest_x - curr_x
    y = dest_y - curr_y
    theta = atan2(y, x)*(180/pi) - curr_theta
    
    pub.publish(x, y, theta)
    rospy.sleep(0.1)

##########################################################

if __name__ == '__main__':
    print("[ INFO] Initialized Optimization Node.")
    while not rospy.is_shutdown():
        get_xy_pose()
        get_dest_xy_pose()
        dist_to_go()
        # print("x : ", dest_x - curr_x)
        # print("y : ", dest_y - curr_y)
        # print("theta : ", dest_theta - curr_theta, "\n\n")
        # print("displacement : ", ((dest_x - curr_x)**2 + (dest_y - curr_y)**2)**0.5)
