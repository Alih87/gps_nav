#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
import subprocess, can, signal
import Jetson.GPIO as GPIO

GPIO.setmode(GPIO.BOARD)
GPIO.setup(7, GPIO.IN)
already_run = False
channel = "can0"

bus = can.interface.Bus(channel=channel, bustype='socketcan')
sys.stdout.write("[INFO] Bringing up CAN...\n")
rospy.sleep(0.5)
subprocess.Popen(["rosrun","scout_bringup","bringup_can2usb.bash"])
rospy.sleep(1)

sys.stdout.write("[INFO] Running Launch file...\n")

if __name__ == "__main__":
	while not rospy.is_shutdown():
		inp = GPIO.input(7)
		if not bool(inp) and not already_run:
			rospy.sleep(0.5)
			already_run = True
			child = subprocess.Popen(["roslaunch","gps_nav","scout_nav.launch"])

		elif bool(inp) and already_run:
			rospy.sleep(0.5)
			already_run = False
			child.send_signal(signal.SIGINT)
	GPIO.cleanup()


