#!/usr/bin/env python

import can, time
from collections import deque
from struct import pack
#from gps_nav.CMD_BIN import CMD_BIN
from can import interface

class CAN_ISOBUS(object):
	def __init__(self):
		#self.arb_id = arb_id
		self.DATA = [0, 0, 0, 0, 0, 0]
		#self.bins = CMD_BIN
		#self.incoming = ""

		self.REMOTE_CTRL = 0b00000000
		self.AUTO_CTRL = 0b00000001
		self.SET_WAY_PNT = 0b00000010
		self.QUALITY_CHECK = 0b00000011
		self.SHRIMP_CHECK = 0b00000100

		self.MONITOR = 0x201
		self.SET = 0x101
		#print(self.REMOTE_CTRL)
		
		
	def set_track_speed(self, lt=0, rt=0):
		if int(lt) > 100:
			lt = 100
		if int(rt) > 100:
			rt = 100
		if int(lt) < 0:
			lt = 0
		if int(rt) < 0:
			rt = 0
		if (lt <= 100 and lt >= 0) and (rt <= 100 and rt >= 0):
			self.DATA[2] = hex(int(lt))[2:]
			self.DATA[3] = hex(int(rt))[2:]
		if len(self.DATA[2]) < 2:
			self.DATA[2] = '0'+self.DATA[2]
		if len(self.DATA[3]) < 2:
			self.DATA[3] = '0'+self.DATA[3]

	def set_feed_spread(self, sp=0):
		if int(sp) > 100:
			lt = 100
		if int(sp) < 0:
			lt = 0
		if sp <= 100 and sp >= 0:
			self.DATA[4] =  hex(int(sp))[2:]
		if len(self.DATA[4]) < 2:
			self.DATA[4] = '0'+self.DATA[4]	

	def set_feed_amt(self, amt=0):
		if int(amt) > 100:
			lt = 100
		if int(amt) < 0:
			lt = 0
		if amt <= 100 and amt >= 0:
			self.DATA[5] =  hex(int(amt))[2:]
		if len(self.DATA[5]) < 2:
			self.DATA[5] = '0'+self.DATA[5]

	def mode_byte(self, engine=0, winch=0, STOP=0, mode=0):
		'''
		mode = 0 : REMOTE CTRL
		mode = 1 : AUTONOMOUS CTRL
		mode = 2 : WAYPOINT SET
		mode = 3 : WATER QLTY MEASURE
		mode = 4 : SHRIMP MEASURE

		engine = 2 : engine start
		engine = 1 : engine stop
		engine = 0 : Do nothing

		winch = 2 : winch wound
		winch = 1 : winch unwound
		winch = 0 : Do nothing

		By default the mode is REMOTE_CONTROL
		'''
		byte = 0b00000000
		if engine == 2:
			byte += 0b10000000
		if engine == 1:
			byte += 0b01000000
		if engine != 2 and engine != 1:
			byte += 0b00000000

		if winch == 2:
			byte += 0b00100000
		if winch == 1:
			byte += 0b00010000
		if winch != 2 and winch != 1:
			byte += 0b00000000

		if bool(STOP):
			byte += 0b00001000
		if not bool(STOP):
			byte += 0b00000000
		
		byte += mode

		self.DATA[0] = hex(int(byte))[2:]
		if len(self.DATA[0]) < 2:
			self.DATA[0] = '0'+self.DATA[0]

	def dir_byte(self, forward=2, measure=0, lift_up=0, feeder=0):
		'''
		Forward = 2 : Forward Movement
		Forward = 1 : Backward Movement
		Forward = 0 : Stop

		measure = 2 : start measure
		measure = 1 : stop measure
		measure = 0 : N/A

		lift_up = 2 : Lift up
		lift_up = 1 : Lift down
		lift_up = 0 : N/A
		'''
		byte = 0b00000000
		if forward == 2:
			byte += 0b10000000
		if forward == 1:
			byte += 0b01000000
		if forward != 2 and forward != 1:
			byte += 0b00000000
		
		if measure == 2:
			byte += 0b00100000
		if measure == 1:
			byte += 0b00010000
		if measure != 2 and measure != 1:
			byte += 0b00000000

		if lift_up == 2:
			byte += 0b00001000
		if lift_up == 1:
			byte += 0b00000100
		if lift_up != 2 and lift_up != 1:
			byte += 0b00000000
		
		if bool(feeder):
			byte += 0b00000010
		if not bool(feeder):
			byte += 0b00000000

		self.DATA[1] = hex(int(byte))[2:]
		if len(self.DATA[1]) < 2:
			self.DATA[1] = '0'+self.DATA[1]

	def set_cmd(self, cmd_dict):
		engine = cmd_dict['engine']
		winch = cmd_dict['winch']
		stop = cmd_dict['stop']
		mode = cmd_dict['mode']

		forward = cmd_dict['forward']
		measure = cmd_dict['measure']
		lift_up = cmd_dict['lift_up']
		feeder = cmd_dict['feeder']

		LT_speed = cmd_dict['LT']
		RT_speed = cmd_dict['RT']
		FEED_sp = cmd_dict['F_sp']
		FEED_amt = cmd_dict['F_amt']

		self.mode_byte(engine=engine, winch=winch, STOP=stop, mode=mode)
		self.dir_byte(forward=forward, measure=measure, lift_up=lift_up, feeder=feeder)
		self.set_track_speed(LT_speed, RT_speed)
		self.set_feed_spread(FEED_sp)
		self.set_feed_amt(FEED_amt)
		#print(self.DATA)
		self.DATA = 't1018' + ''.join(self.DATA) + '0' + '0' + '0'+ '0' + '\r'

		return self.DATA

		#print(self.DATA)
	'''
	def battery(self, DATA):
		pass

	def send_frame(self, bus):
		if self.arb_id == self.SET:
			msg = can.Message(arbitration_id=self.arb_id, data=self.DATA)
			bus.send(msg)

	def rcv_frame(self, bus):
		if self.arb_id == self.MONITOR:
			msg = bus.recv()
			self.incoming.append(str(msg))
	'''

##if __name__ == '__main__':



#	isobus = CAN_ISOBUS()
#	cmd_dict = {
#			'engine' : 0,
#			'winch' : 0,
#			'stop' :  0,
#			'mode' : 0,
#
#			'forward': 2,
#			'measure': 0,
#			'lift_up': 0,
#			'feeder' : 0,
#			
#			'LT': 0,
#			'RT':0,
#			'F_sp':0,
#			'F_amt':0
#			}
	
	#isobus.set_cmd(cmd_dict)
#	can.rc['interface'] = 'socketcan'
#	can.rc['channel'] = 'vcan0'
#	can.rc['bitrate'] = 1000000
#	
#	REMOTE_CTRL = 0b00000000
#	AUTO_CTRL = 0b00000001
#	SET_WAY_PNT = 0b00000010
#	QUALITY_CHECK = 0b00000011
#	SHRIMP_CHECK = 0b00000100
	
#	MONITOR = 0x201
#	SET = 0x101
#	bus = interface.Bus()
	
#	canbus = CAN_ISOBUS(SET)

#	canbus.mode_byte(engine=1, mode=AUTO_CTRL)
#	canbus.dir_byte()

#	canbus.set_track_speed(lt=20, rt=20)
#	canbus.set_feed_spread(sp = 20)
#	canbus.set_feed_amt(amt=20)
	
#	canbus.send_frame(bus)
	

	#canbus1 = CAN_ISOBUS(MONITOR)	
	#canbus1.rcv_frame(bus)

