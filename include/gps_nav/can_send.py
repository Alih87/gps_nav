#!/usr/bin/env python

import serial
from struct import pack
from gps_nav.boat_can import CAN_ISOBUS

#read_cmd = "t101811111000\r".encode()

class serial_can(object):
	def __init__(self, serial_port):
		self._port = serial_port
		self.data = ""

	def to_hex(x):
		return hex(int(x))

	def send_rcv(self, msg):
		self.data = ""
		for b in msg:
			if type(b) is str:
				self._port.write(b)

		while(True):
			if self._port.inWaiting() > 0:
				self.data += self._port.read()
			if len(self.data) > 41:
				break
		print(self.data)

	def mode_byte_status(self, byte):
		mode = (byte >> 6) & 0xf
		if mode == 0:
			mode = 'REMOTE_CTRL'
		elif mode == 1:
			mode = 'AUTO_CTRL'
		elif mode == 2:
			mode = 'WAYPT_SET'
		elif mode == 3:
			mode = 'WATER_QLTY'
		elif mode == 4:
			mode = 'SHRIMP_MS'
		else:
			mode = 'INVALID'

		stop = bool((byte >> 3) & 0x01)
		winch_uw = bool((byte >> 4) & 0x01)
		winch_w = bool((byte >> 5) & 0x01)
		engine_s = bool((byte >> 6) & 0x01)
		engine_r = bool((byte >> 7) & 0x01)

		return {
			'mode' : mode,
			'stop' : stop,
			'winch_uw': winch_uw,
			'winch_w': winch_w,
			'engine_s': engine_s,
			'engine_r': engine_r
			}

	def dir_byte_status(self, byte):
		feeder = bool((byte >> 1) & 0x01)
		lift_d = bool((byte >> 2) & 0x01)
		lift_u = bool((byte >> 3) & 0x01)
		measure_stp = bool((byte >> 4) & 0x01)
		measure_str = bool((byte >> 5) & 0x01)
		fb_ctrl = (byte >> 6) & 0x03
		print(fb_ctrl)
		if fb_ctrl == 0:
			fb_ctrl = 'stop'
		elif fb_ctrl == 1:
			fb_ctrl = 'backward'
		elif fb_ctrl == 2:
			fb_ctrl = 'forward'

		return {
			'feeder' : feeder,
			'lift_d' : lift_d,
			'lift_u': lift_u,
			'measure_stp': measure_stp,
			'measure_str': measure_str,
			'fb_ctrl': fb_ctrl
			}

	def track_sp_status(self, byte_3, byte_4):
		return {
			'LT_sp' : byte_3,
			'RT_sp' : byte_4
			}

	def feeder_spr_status(self, byte_5):
		return {
			'F_spr' : byte_5
			}

	def feeder_amt_status(self, byte_6):
		return {
			'F_amt' : byte_6
			}

	def battery_status(self, byte_7, byte_8):
		total = (byte_8 << 8) + byte_7
		if total < 535 and total > 415:
			perc = int((total - 420)*100/(540 - 420))
		elif total >= 535:
			perc = 100
		elif total <= 415:
			perc = 0
			
		return {
			'battery' : perc
			}

	def get_status(self):
		data = self.data[5:]
		bytes = []
		for i in range(0, len(data)-1, 2):
			bytes.append(data[i:i+2])
		#print(bytes)
		bytes_hex = map(lambda x: int(x, base=16), bytes)
		#print(bytes_hex)
		mode_st = self.mode_byte_status(bytes_hex[0])
		dir_st = self.dir_byte_status(bytes_hex[1])
		track_sp_st = self.track_sp_status(bytes_hex[2], bytes_hex[3])
		feed_spr_st = self.feeder_spr_status(bytes_hex[4])
		feed_amt_st = self.feeder_amt_status(bytes_hex[5])
		bat_st = self.battery_status(bytes_hex[6], bytes_hex[7])
		
		print(mode_st, dir_st, track_sp_st, feed_spr_st, feed_amt_st, bat_st)

		

if __name__ == '__main__':

	REMOTE_CTRL = 0b00000000
	AUTO_CTRL = 0b00000001
	SET_WAY_PNT = 0b00000010
	QUALITY_CHECK = 0b00000011
	SHRIMP_CHECK = 0b00000100

	MONITOR = 0x102
	SET = 0x101

	canbus = CAN_ISOBUS()

	cmd_dict = {
			'engine' : 2,
			'winch' : 0,
			'stop' :  0,
			'mode' : AUTO_CTRL,

			'forward': 0,
			'measure': 0,
			'lift_up': 0,
			'feeder' : 0,

			'LT': 0,
			'RT': 0,
			'F_sp':0,
			'F_amt':0
		    }

	cmd = str(canbus.set_cmd(cmd_dict)).encode()
	print(cmd)
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
	slcan.send_rcv(cmd)
	slcan.get_status()			


