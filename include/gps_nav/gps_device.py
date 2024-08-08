#!/usr/bin/env python
import time
from collections import deque
from struct import pack
import serial

class GPS(object):
	def __init__(self, serial_port):
		print("[INFO] Connected to the RTK2U Device.")
		self.port = serial_port
		self.buf = "".encode()
		self.head = "$GNGGA".encode()
		self.msgs = deque(maxlen=4)
		self.has_head = False
		self.end = "\r\n".encode()
		self.frame_k = ['UTC_fix', 'dir_lat', 'lat', 'dir_lon', 'lon',
				 'gps_qty', 'num_SV', 'hdop', 'orth_height',
				 'm_orth_height', 'geoid_sep', 'm_geoid_sep',
				 'DGPS_age', 'refer_id', 'chksum']
	
	def read(self):
		'''
		Parse the incoming bytes into separate bytes.
		'''
		wait_idx = 0
		while True:
			if self.port.inWaiting() > 0:
				wait_idx = 0
				self.buf += self.port.read()
				#print(self.buf)
				if self.head in self.buf:
					self.buf = "".encode()
					self.has_head = True

				if self.has_head and "\r".encode() in self.buf:
					self.msgs.append(self.buf[:-1])
					self.buf = "".encode()
					break

				if len(self.buf) >= 999:
					self.buf = "".encode()
					break

			else:
				wait_idx += 1
				if wait_idx == 200000000:
					wait_idx = 0
					break

	def parse(self):
		if len(self.msgs) > 0:
			msg = self.msgs[-1].decode()
			msg_list = msg.split(',')
			msg_list[-1] = msg_list[-1][1:]

			#print(dict(zip(self.frame_k, msg_list)))		

			return dict(zip(self.frame_k, msg_list))

		else:
			return dict()


if __name__ == '__main__':
	CONNECTED = False
	lat_lons = list()
	try:
		serial_port = serial.Serial(
			port="/dev/ttyUSB1",
			baudrate=115200,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE
					   )
		CONNECTED = True
		gps = GPS(serial_port)
		while True:
			gps.read()
			loc_dict = gps.parse()
			lat_lons.append((loc_dict['lat'], loc_dict['lon']))
	except:
		with open("/home/scout/Documents/sandesh_dynamic_test.txt", 'w') as f:
			for lt, ln in lat_lons:
				f.write(lt+","+ln+"\n")

			f.close()
		print("[INFO] Data saved.")

