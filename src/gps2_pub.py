#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial, sys
from gps_nav.msg import latlon_gps, heading_ang
from gps_nav.gps_device import GPS

class gps2_node(object):
	def __init__(self):
		self.X, self.Y, self.PORT = 0, 0, 0
	def loc_pub(self, x, y):
		rospy.init_node('dgps2', anonymous=False)
		pub = rospy.Publisher('gps_pos2', latlon_gps, queue_size=10)
		try:
			X, Y = float(x)/100.0, float(y)/100.0
		except:
			X, Y = 0, 0
		pub.publish(X,Y)
		rospy.sleep(0.01)

	def port_callback(self, data):
		if data.angle is not None:
			self.PORT = int(data.angle)

	def sub_port_num(self):
		rospy.init_node('dgps2', anonymous=False)
		rospy.Subscriber('port_num', heading_ang, self.port_callback)
		rospy.sleep(0.01)


if __name__ == '__main__':
	gps2_obj = gps2_node()
	for _ in range(10):
		gps2_obj.sub_port_num()

	if int(gps2_obj.PORT) == 1:
		sys.stdout.write("[INFO] Port conflict. Restart the System.")
		while(True):
			pass

	try:
		serial_port = serial.Serial(
			port="/dev/ttyUSB1",
			baudrate=115200,
			bytesize=serial.EIGHTBITS,
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE
					   )
		if not serial_port.isOpen():
			pass
		else:
			gps = GPS(serial_port)
			sys.stdout.write("\n[INFO] Connection established at port USB1"+"\n")

		while not rospy.is_shutdown():
			ret = gps.read()
			frame = gps.parse()
			if len(list(frame.keys())) == 0:
				break
			gps2_obj.loc_pub(frame['dir_lat'], frame['dir_lon'])

		
	except:
		sys.stdout.write("[INFO] GPS2 initialization failed. Restart the System.")

