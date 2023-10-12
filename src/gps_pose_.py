#!/usr/bin/env python3
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from gps_nav.msg import coordinates, pose_xy, latlon_gps, heading_ang
#from sbg_driver.msg import SbgGpsPos, SbgMag
from nav_msgs.msg import Odometry
from utm import from_latlon
from math import atan, atan2, pi

CENTER = (388731.70, 3974424.49)
ZONE = ''
X = 0
Y = 0
HEADING = 0
# HEADING = deque(maxlen=5)

def get_utm(data):
    global X, Y
    lat, long = data.lat, data.lon
    X, Y, zo, ne = from_latlon(lat, long)
    X, Y = X - CENTER[0], Y - CENTER[1]
    X, Y = X, Y
    # ZONE = str(zo)+ne

def get_heading(data):
    angle = data.angle
    global HEADING
    HEADING = angle

def gps_sub():
    rospy.init_node('gps_pose', anonymous=False)
    rospy.Subscriber('gps_pos', latlon_gps, get_utm)
    rospy.sleep(0.01)

def mag_sub():
    rospy.init_node('gps_pose', anonymous=False)
    rospy.Subscriber('um7_heading', heading_ang, get_heading)
    rospy.sleep(0.01)

def utm_pub():
    global X, Y, ZONE
    rospy.init_node('gps_pose', anonymous=False)
    pub = rospy.Publisher('odom_pose', coordinates, queue_size=10)
    pub.publish(X,Y,HEADING)
    rospy.sleep(0.01)

############### USING SCOUT ODOMETER ##################

def get_theta(q):
    siny_cosp = 2*(q.w*q.z + q.x*q.z)
    cosy_cosp = 1 - 2*(q.y*q.y + q.z*q.z)

    return atan2(siny_cosp, cosy_cosp)*(180/pi)

def get_state(data):
    pos = data.pose.pose.position
    quat = data.pose.pose.orientation
    global X, Y, HEADING
    X, Y = pos.x, pos.y
    HEADING = get_theta(quat)

    # print("Position : ", X, Y)
    # print("Angle : ", HEADING)

def odom_pub():
    rospy.init_node('gps_pose', anonymous=False)
    pub = rospy.Publisher('odom_pose', pose_xy, queue_size=30)
    pub.publish(X,Y,HEADING)
    rospy.sleep(0.1)

def odom_sub():
    rospy.init_node('gps_pose', anonymous=False)
    rospy.Subscriber('/odom', Odometry, get_state)
    rospy.sleep(0.1)

#######################################################

if __name__== '__main__':
	print("[ INFO] Initialized GPS and Heading Node.")
	while not rospy.is_shutdown():
		gps_sub()
		mag_sub()
		utm_pub()

	     # Using Scout Odometer
		# odom_sub()
		# odom_pub()

