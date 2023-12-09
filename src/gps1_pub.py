#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial, sys
from gps_nav.msg import latlon_gps, heading_ang
from gps_nav.gps_device import GPS

class gps1_node(object):
	def __init__(self):
		self.X, self.Y = 0, 0
	def loc_pub(self, x, y):
		rospy.init_node('dgps1', anonymous=False)
		pub = rospy.Publisher('gps_pos1', latlon_gps, queue_size=10)
		try:
			self.X, self.Y = float(x)/100.0, float(y)/100.0
		except:
			self.X, self.Y = 0, 0
		pub.publish(self.X, self.Y)
		rospy.sleep(0.01)

	def pub_port_num(self, port_num):
		rospy.init_node('dgps1', anonymous=False)
		pub = rospy.Publisher('port_num', heading_ang, queue_size=10)
		pub.publish(port_num)
		rospy.sleep(0.01)

if __name__ == '__main__':
	gps1_obj = gps1_node()
	rospy.sleep(1)
	try:
		serial_port = serial.Serial(
			port="/dev/ttyUSB0",
			baudrate=115200,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE
					   )
		if not serial_port.isOpen():
			pass
		else:
			gps = GPS(serial_port)
			sys.stdout.write("\n[INFO] Connection established at port USB0"+"\n")

		while not rospy.is_shutdown():
			ret = gps.read()
			frame = gps.parse()
			if len(list(frame.keys())) == 0:
				break
			gps1_obj.pub_port_num(0)
			gps1_obj.loc_pub(frame['dir_lat'], frame['dir_lon'])

	except:
		sys.stdout.write("[INFO] GPS1 initialization failed. Restart the System.\n")

