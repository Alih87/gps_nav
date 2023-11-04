#!/usr/bin/env python
import time
from collections import deque
from struct import pack

class GPS(object):
	def __init__(self, serial_port):
		print("[INFO] Connected to the RTK2U Device.")
		self.port = serial_port
		self.buf = "".encode()
		self.head = "$GNGGA".encode()
		self.msgs = deque(maxlen=4)
		self.has_head = False
		self.end = "\r\n".encode()
		self.frame_k = ['UTC_fix', 'lat', 'dir_lat', 'lon', 'dir_lon',
				 'gps_qty', 'num_SV', 'hdop', 'orth_height',
				 'm_orth_height', 'geoid_sep', 'm_geoid_sep',
				 'DGPS_age', 'refer_id', 'chksum']
	
	def read(self):
		'''
		Parse the incoming bytes into separate bytes.
		'''
		wait_idx = 0
		try:
			while True:
				if self.port.inWaiting() > 0:
					self.buf += self.port.read()
					if self.has_head and "\r".encode() in self.buf:
						self.msgs.append(self.buf[:-1])
						self.buf = "".encode()
						break
					
					if self.head in self.buf:
						self.buf = "".encode()
						self.has_head = True
						
					#if not (self.head in self.buf) and len(self.buf) >= len(self.head) and not found:
					#	self.buf = "".encode()
					#	return True
					#if wait_idx >= 50:
					#	self.buf = "".encode()
					#	wait_idx = 0
					#	return True
										
		except KeyboardInterrupt:
			print("\nExiting...\n")

	def parse(self):
		msg = self.msgs[-1].decode()
		msg_list = msg.split(',')
		msg_list[-1] = msg_list[-1][1:]

		#print(dict(zip(self.frame_k, msg_list)))		

		return dict(zip(self.frame_k, msg_list))


#if __name__ == '__main__':
#	CONNECTED = False
#	for i in range(21):
#		if i == 21 and not CONNECTED:
#			print("\n[INFO] No Port found!\n")
#			break
#		try:
#			serial_port = serial.Serial(
#				port="/dev/ttyUSB"+str(i),
#				baudrate=115200,
#				bytesize=serial.EIGHTBITS,
#				parity=serial.PARITY_NONE,
#				stopbits=serial.STOPBITS_ONE
#						   )
#			print("\n[INFO] Connection established at port USB"+str(i))
#			CONNECTED = True
#			gps = GPS(serial_port)
#			gps.read()
#			print(gps.parse())
#		except:
#s			print('Here at '+str(i))

