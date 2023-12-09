#!/usr/bin/env python
import subprocess, sys, can, signal
import Jetson.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.IN)
already_run = False
channel = "can0"

bus = can.interface.Bus(channel=channel, bustype='socketcan')
sys.stdout.write("[INFO] Bringing up CAN...\n")
sleep(0.5)
subprocess.Popen(["rosrun","scout_bringup","bringup_can2usb.bash"])
sleep(1)

sys.stdout.write("[INFO] Running Launch file...\n")

while True:
	inp = GPIO.input(7)
	if not bool(inp) and not already_run:
		sleep(0.5)
		already_run = True
		child = subprocess.Popen(["roslaunch","gps_nav","scout_nav.launch"])

	elif bool(inp) and already_run:
		sleep(0.5)
		already_run = False
		child.send_signal(signal.SIGINT)
GPIO.cleanup()
		

