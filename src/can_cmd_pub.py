#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy
import sys
from gps_nav.boat_can import CAN_ISOBUS
from gps_nav.can_send import serial_can
from gps_nav.msg import can_pose

REMOTE_CTRL = 0b00000000
AUTO_CTRL = 0b00000001
SET_WAY_PNT = 0b00000010
QUALITY_CHECK = 0b00000011
SHRIMP_CHECK = 0b00000100

engine = 2
winch = 0
stop = 0
mode = AUTO_CTRL

forward, measure, lift_up, feeder = 0, 0, 0, 0


LT, RT, F_sp, F_amt = 0, 0, 0, 0

cmd_dict = {
		'engine' : engine,
		'winch' : winch,
		'stop' :  stop,
		'mode' : AUTO_CTRL,

		'forward': forward,
		'measure': measure,
		'lift_up': lift_up,
		'feeder' : feeder,

		'LT': LT,
		'RT': RT,
		'F_sp': F_sp,
		'F_amt': F_amt
	    }

def get_can_pose(data):
	global RT, LT, cmd_dict
	RT, LT = data.rt, data.lt
	if RT > LT:
		cmd_dict['RT'], cmd_dict['LT'] = RT, 0
	elif RT < LT:
		cmd_dict['RT'], cmd_dict['LT'] = 0, LT

def can_pose_sub():
	rospy.init_node('cmd_node', anonymous=False)
    	rospy.Subscriber("cmd_vel", can_pose, get_can_pose)


if __name__ == '__main__':
	canbus = CAN_ISOBUS()
	for i in range(11):
		try:
			serial_port = serial.Serial(
				port="/dev/ttyUSB"+str(i),
				baudrate=115200,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE)
			print("[INFO] Connected on port "+str(i))
			break

		except:
			if i == 10:
				print('[INFO] Could not connect to serial device.')
	
	slcan = serial_can(serial_port)
	
