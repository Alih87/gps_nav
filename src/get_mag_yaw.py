#!/usr/bin/env python
import roslib
import tf.transformations; roslib.load_manifest('gps_nav')
import rospy, sys
from sensor_msgs.msg import Imu

def get_mag_yaw():
    enu_data = Imu()
    
    # Copy header
    enu_data.header = ned_data.header
    
    
    return enu_data

def imu_callback(data):
    enu_data = ned_to_enu(data)
    pub.publish(enu_data)

if __name__== '__main__':
    rospy.init_node("ned_to_enu")
    rate = rospy.Rate(5)
    pub = rospy.Publisher('new_data', Imu, queue_size=4)
    rospy.Subscriber('imu/data', Imu, imu_callback)
    rospy.spin()
