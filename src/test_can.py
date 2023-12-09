#!/usr/bin/env python

import serial
from struct import pack

for i in range(10):
	try:
		serial_port = serial.Serial(
			port="/dev/ttyUSB"+str(i),
			baudrate=115200,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE)
		print("[INFO] Connected on port "+str(i))

	except:
		pass
	
	read_cmd = "t101811111000\r".encode()
	for b in read_cmd:
		if type(b) is str:
			serial_port.write(b.encode())
		else:
			serial_port.write(bytes(int(b)))

	while True:
		if serial_port.inWaiting() > 0:
			data = serial_port.read()
			print(data)
			

