#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy
import sys
from gps_nav.boat_can import CAN_ISOBUS
from gps_nav.can_send import serial_can
from gps_nav.msg import can_pose, status

global called
called = False

def finish_callback(data):
	global cmd_dict, called
	if data:
		cmd_dict['RT'], cmd_dict['LT'] = 0, 0

	called = True

def get_can_pose(data):
	global RT, LT, cmd_dict, called
	RT, LT = data.rt, data.lt
	if RT > LT:
		cmd_dict['RT'], cmd_dict['LT'] = RT, 0
	elif RT < LT:
		cmd_dict['RT'], cmd_dict['LT'] = 0, LT

	called = True

def can_pose_sub():
	global called
	rospy.init_node('cmd_node', anonymous=False)
    	rospy.Subscriber("cmd_vel", can_pose, get_can_pose)

	if called:
		called = False
		return True
	else:
		return False
	rospy.sleep(0.01)

def finish_flag_sub():
	global called
	rospy.init_node('cmd_node', anonymous=False)
	rospy.Subscriber("stopper", flag, finish_callback)
	rospy.sleep(0.01)
	
	if called:
		called = False
		return True
	else:
		return False

def pub_status(cmd_dict):
    rospy.init_node('cmd_node', anonymous=False)
    pub = rospy.Publisher('boat_status', status, queue_size=10)

    engine, winch, stop, mode, forward, measure, lift_up, feeder, LT, RT, F_sp, F_amt = str(cmd_dict['engine']), str(cmd_dict['winch']), str(cmd_dict['stop']), str(cmd_dict['mode']), str(cmd_dict['forward']), str(cmd_dict['measure']), str(cmd_dict['lift_up']), str(cmd_dict['feeder']), str(cmd_dict['LT']), str(cmd_dict['RT']), str(cmd_dict['F_sp']), str(cmd_dict['F_amt'])

    pub.publish(engine, winch, stop, mode, forward, measure, lift_up, feeder, LT, RT, F_sp, F_amt)
    rospy.sleep(0.01)

if __name__ == '__main__':
	canbus = CAN_ISOBUS()

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
	
	s_can = serial_can(serial_port)
	while not rospy.is_shutdown():
		change = can_pose_sub()
		change = finish_flag_sub()

		cmd = canbus.set_cmd(cmd_dict)
		if change:
			pass
			#slcan.send_rcv(cmd)
			#slcan.get_status()
			#pub_status(cmd_dict)
		
