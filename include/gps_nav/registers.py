#!/usr/bin/env python3
from math import pi, atan, atan2

class Registers(object):
	def __init__(self):
		self.DREG_GYRO_RAW_XY = 86
		self.DREG_GYRO_RAW_Z = 87
		self.DREG_GYRO_RAW_TIME = 88
		self.DREG_ACCEL_RAW_XY = 89
		self.DREG_ACCEL_RAW_Z = 90
		self.DREG_ACCEL_RAW_TIME = 91
		self.DREG_MAG_RAW_XY = 92
		self.DREG_MAG_RAW_Z = 93
		self.DREG_MAG_RAW_TIME = 94
		self.DREG_TEMPERATURE = 95
		self.DREG_TEMPERATURE_TIME = 96

		self.start = 86
		self.end = 96

	def sign(self, x):
		if x>187:
			return -1
		else:
			return 1

	def calculate_angle(self, y, x):
		if x > 0:
			return atan(y/x)
		if x==0 and y > 0:
			return pi/2
		if x==0 and y < 0:
			return -pi/2
		if x < 0 and y >= 0:
			return atan(y/x) + pi
		if x < 0 and y < 0:
			return atan(y/x) - pi

	def calculate_angle2(self, y, x):
		return atan2(y,x)

	def take_2s_comp(self, x, bits=16):
		if (x & (1 << (bits-1))) is not 0:
			x = x - (1 << bits)
		return x

	def read_packet(self, data, typ='heading'):
		if typ == 'heading':
			#print(data)
			y_l = data[int(self.DREG_MAG_RAW_XY-self.start)*4+1]
			x_l = data[int(self.DREG_MAG_RAW_XY-self.start)*4+3]
			z_l = data[int(self.DREG_MAG_RAW_Z-self.start)*4+3]
			y_u = data[int(self.DREG_MAG_RAW_XY-self.start)*4]
			x_u = data[int(self.DREG_MAG_RAW_XY-self.start)*4+2]
			z_u = data[int(self.DREG_MAG_RAW_Z-self.start)*4+2]
			
			x = self.take_2s_comp((x_u << 8) | x_l)
			y = self.take_2s_comp((y_u << 8) | y_l)
			#print(x,y,"\n")
			theta = self.calculate_angle2(y, x)

			return {'theta':float(theta*(180/pi))}


