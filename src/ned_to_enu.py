#!/usr/bin/env python
import roslib
import tf.transformations; roslib.load_manifest('gps_nav')
import rospy, sys
from sensor_msgs.msg import Imu

def ned_to_enu(ned_data):
    enu_data = Imu()
    
    # Copy header
    enu_data.header = ned_data.header
    
    # Convert orientation (assuming quaternion representation)
    enu_data.orientation.x = ned_data.orientation.y
    enu_data.orientation.y = ned_data.orientation.x
    enu_data.orientation.z = -ned_data.orientation.z
    enu_data.orientation.w = ned_data.orientation.w
    
    # Convert angular velocity
    enu_data.angular_velocity.x = ned_data.angular_velocity.y
    enu_data.angular_velocity.y = ned_data.angular_velocity.x
    enu_data.angular_velocity.z = -ned_data.angular_velocity.z
    
    # Convert linear acceleration
    enu_data.linear_acceleration.x = ned_data.linear_acceleration.y
    enu_data.linear_acceleration.y = ned_data.linear_acceleration.x
    enu_data.linear_acceleration.z = -ned_data.linear_acceleration.z
    
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
