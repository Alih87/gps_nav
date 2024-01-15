#!/usr/bin/env python

import serial, time
from collections import deque
from gps_nav.registers import Registers
from struct import pack


class UM7():
	def __init__(self, serial_port):
		self.serial = serial_port

	def read(self):
		'''
		Parse the incoming bytes into separate bytes.
		'''
		while(True):
			try:
				while True:
					if self.port.inWaiting() > 0:
						data = self.port.read()
						self.buf += data
						print(data)

			except:
				print("Something's wrong\n")

if __name__ == '__main__':
	serial_port = serial.Serial(
			port="/dev/ttyUSB0",
			baudrate=115200,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE)

	um7_obj = UM7(serial_port)
	um7_obj.read()

