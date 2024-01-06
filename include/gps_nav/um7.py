#!/usr/bin/env python

import serial, time
from collections import deque
from gps_nav.registers import Registers
from struct import pack

class UM7(object):
	def __init__(self, serial_port, reg_obj):
		self.header_found = False
		self.port = serial_port
		self.buf = "".encode()
		self.packet = []
		self.full_packets = deque(maxlen=5)
		
		self.HE = bytearray('s')[0]
		self.AD = bytearray('n')[0]
		self.ER = bytearray('p')[0]

		self.HEADER = self.HE + self.AD + self.ER

		self.reg = reg_obj
	
	def read(self):
		'''
		Parse the incoming bytes into separate bytes.
		'''
		try:
			while True:
				if self.port.inWaiting() > 0:
					data = self.port.read()
					self.buf += data
					if self.header_found and "snp".encode() in self.buf:
						self.buf = "".encode()
						self.header_found = False
						self.full_packets.append(self.packet[:-2])
						self.packet = []
						break
					elif "snp".encode() in self.buf:
						header = self.buf
						self.buf = "".encode()
						self.header_found = True
					elif self.header_found:
						self.packet.append(bytearray(data)[0])

		except KeyboardInterrupt:
			print("\nExiting Program\n")

	def READ_REG(self, reg_addr, is_batch=False, n_regs=None):
		'''
		Construct the read command value
		'''
		if is_batch:
			PT_byte = int(64)
			PT_byte = PT_byte + (n_regs << 2)
			checksum = bin(self.HEADER + PT_byte + reg_addr)
			data = int(0)
			packet = [pack('B',self.HE),pack('B',self.AD),pack('B',self.ER),pack('B',PT_byte),pack('B',reg_addr), pack('B', (checksum >> 8)&0xff),pack('B', checksum&0xff)]
			return packet
		else:
			PT_byte = int(0)
			data = int(0)
			checksum = int(bin(self.HEADER + PT_byte + reg_addr), base=2)
			packet = pack('B',self.HE)+pack('B',self.AD)+pack('B',self.ER)+pack('B',PT_byte)+pack('B',int(reg_addr))+pack('B', (checksum >> 8)&0xff)+pack('B', checksum&0xff)
			return packet

	def send_read_cmd(self, reg_addr):
		'''
		Send cmd to read a specified register.
		'''
		read_cmd = self.READ_REG(reg_addr)
		self.port.write(bytearray(str(read_cmd)))
		response = self.parse()
		return response
		
		
	def parse(self, check_status_of=None):
		'''
		Read the packets being received.
		Pass register address to check the status of previous write operation.
		'''
		self.read()
		msg = self.full_packets[-1]
		#print(msg)
		try:
			calc_checksum = self.HEADER + msg[0] + msg[1] + sum(msg[2:4*(msg[0]>>2 & 0x0f)+2])
			rcv_checksum = msg[2+4*(msg[0]>>2 & 0x0f)] << 8 | msg[2+4*(msg[0]>>2 & 0x0f)+1]
			assert calc_checksum == rcv_checksum
		except:
			print("[INFO] Bad Checksum.")
			pass

		has_data = (msg[0] >> 7) & 1
		is_batch = (msg[0] >> 6) & 1
		CF = msg[0] & 1
		data_length = msg[0]>>2 & 0x0f
		data = 0
		address = msg[1]

		if has_data and check_status_of is None:
			if is_batch:
				num_bytes = data_length * 4
				data = msg[2:num_bytes+2]
				return {
				'addr': address,
				'data': data
				}
			else:
				num_bytes = 4
				data = msg[2:num_bytes+2]
				return {
				'addr': address,
				'data': data
				}

		elif not has_data and not is_batch and msg[0] == 0 and check_status_of is not None:
			return check_status_of == address and CF

		else:
			return {
				'addr': address,
				'data': data
				}
	def get_data(self):
		'''
		Extract Accelerometer, Gyro and Magnetometer Data from the Packet
		'''
		packet = self.send_read_cmd(86)
		if packet['addr'] == 86 and len(packet['data']) > 40:
			data = packet['data']
			#print(self.reg.read_packet(data).values())
			return self.reg.read_packet(data)['theta']

if __name__ == '__main__':

	serial_port = serial.Serial(
			port="/dev/ttyTHS1",
			baudrate=9600,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE)
	
	print("[INFO] Establishing UART Communication with UM7 Orientation Device ...")
	#time.sleep(1)

	reg = Registers()
	obj = UM7(serial_port, reg)

	print(obj.get_data())
	#while True:
	#print(obj.send_read_cmd(100))
	#	time.sleep(0.25)
	#obj.parse()

