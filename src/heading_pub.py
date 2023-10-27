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
	pub = rospy.Publisher('um7_heading', heading_ang, queue_size=30)
	global ANGLE
	if angle is not None:
		ANGLE = float(angle)
	pub.publish(ANGLE)
	rospy.sleep(0.01)

if __name__ == '__main__':
	print("\n[INFO] Establishing UART Communication with UM7 Orientation Device ...")
	try:
		for i in range(10):
			serial_port = serial.Serial(
				port="/dev/ttyUSB"+str(i),
				baudrate=115200,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE)
	
			print("Connected")
			break
	except:
		if i ==9:
			print("Can't find port")
		pass

	reg_obj = Registers()
	um7_obj = UM7(serial_port, reg_obj)
	ang_ls = []
	while not rospy.is_shutdown():
		angle = um7_obj.get_data()
		if type(angle) is float:
			ang_ls.append(angle)
		if len(ang_ls) == 15:
			angle = sum(ang_ls)/len(ang_ls)
			ang_ls = []
		#print(angle)
		angle_pub(angle)

