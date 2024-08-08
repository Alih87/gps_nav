#!/usr/bin/env python
import serial
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("command",
                    help="write the command to send (see RTK2U User Manual)")
args = parser.parse_args()


def send_rcv(serial_port):
	cmd = args.command
	serial_port.write("--"+str(cmd)+"\r\n")
	buf = "".encode()
	try:
		while True:
			if serial_port.inWaiting() > 0:
				buf += serial_port.read()

				if "\r".encode() in buf:
					msg = buf[:-1]
					buf = "".encode()
					return msg
					
	except KeyboardInterrupt:
		print("\nExiting...\n")
	

if __name__ == "__main__":
	sys.stdout.write("[INFO] Establishing Serial Connection with RTK2U ...\n")
	serial_port = serial.Serial(
				port="/dev/ttyUSB0",
				baudrate=115200,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE
						   )
	if args.command is None:
		sys.stdout.write("[INFO] Please enter a valid command ...\n")
	else:
		reply = send_rcv(serial_port)
		sys.stdout.write(reply)
