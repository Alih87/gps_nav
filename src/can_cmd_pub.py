#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy
import sys, serial, time
from gps_nav.boat_can import CAN_ISOBUS
from gps_nav.can_send import serial_can
from gps_nav.msg import can_pose, status, flag

#flag = False

class cmd_pub_node(object):
	def __init__(self, cmd_dict):
		self.called = False
		self.cmd_dict = cmd_dict
		self.forward, self.measure, self.lift_up, self.feeder = 0, 0, 0, 0
		self.LT, self.RT, self.F_sp, self.F_amt = 0, 0, 0, 0


	def finish_callback(self, data):
		if data:
			self.cmd_dict['RT'], self.cmd_dict['LT'] = 0, 0

		self.called = True

	def get_can_pose(self, data):
		self.RT, self.LT = data.rt, data.lt
		if self.RT > self.LT:
			self.cmd_dict['RT'], self.cmd_dict['LT'] = self.RT, 0
		elif self.RT < self.LT:
			self.cmd_dict['RT'], self.cmd_dict['LT'] = 0, self.LT

		self.called = True

	def can_pose_sub(self):
		rospy.init_node('cmd_node', anonymous=False)
		rospy.Subscriber("cmd_vel", can_pose, self.get_can_pose)
		rospy.sleep(0.01)

		if self.called:
			self.called = False
			return True
		else:
			return False

	def finish_flag_sub(self):
		rospy.init_node('cmd_node', anonymous=False)
		rospy.Subscriber("stopper", flag, self.finish_callback)
		rospy.sleep(0.01)
		
		if self.called:
			self.called = False
			return True
		else:
			return False

	def pub_status(self):
		rospy.init_node('cmd_node', anonymous=False)
		pub = rospy.Publisher('boat_status', status, queue_size=10)

		engine, winch, stop, mode, forward, measure, lift_up, feeder, LT, RT, F_sp, F_amt = str(self.cmd_dict['engine']), str(self.cmd_dict['winch']), str(self.cmd_dict['stop']), str(self.cmd_dict['mode']),\
																							str(self.cmd_dict['forward']), str(self.cmd_dict['measure']), str(self.cmd_dict['lift_up']), str(self.cmd_dict['feeder']),\
																							str(self.cmd_dict['LT']), str(self.cmd_dict['RT']), str(self.cmd_dict['F_sp']), str(self.cmd_dict['F_amt'])
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

	cmd_obj = cmd_pub_node(cmd_dict=cmd_dict)
	for i in range(11):
		try:
			serial_port = serial.Serial(
				port="/dev/ttyUSB"+str(i),
				baudrate=115200,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE)
			slcan = serial_can(serial_port)
			#time.sleep(0.1)
			while not rospy.is_shutdown():
				cmd_obj.pub_status()
				change = cmd_obj.can_pose_sub()
				change = cmd_obj.finish_flag_sub()
				cmd = str(canbus.set_cmd(cmd_dict)).encode()
				#print(cmd)
				slcan.send_rcv(cmd)
				slcan.get_status()
			print("[INFO] Connected on port "+str(i))
			#break

		except:
			pass
				#while(True):
				#	pass
	
		
