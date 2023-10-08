#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial
from gps_nav.msg import heading_ang
from gps_nav.um7 import UM7
from gps_nav.registers import Registers
global ANGLE
ANGLE = float(0)

def angle_pub(angle):
	rospy.init_node('heading_angle', anonymous=False)
	pub = rospy.Publisher('um7_heading', heading_ang, queue_size=10)
	global ANGLE
	if angle is not None:
		ANGLE = float(angle)
	pub.publish(ANGLE)
	rospy.sleep(0.01)

if __name__ == '__main__':
	print("\n[INFO] Establishing UART Communication with UM7 Orientation Device ...")

	serial_port = serial.Serial(
			port="/dev/ttyTHS1",
			baudrate=115200,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE)

	reg_obj = Registers()
	um7_obj = UM7(serial_port, reg_obj)

	while not rospy.is_shutdown():
	#	print("here")
		angle = um7_obj.get_data()
		angle_pub(angle)

