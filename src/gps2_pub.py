#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial, sys
from gps_nav.msg import latlon_gps, heading_ang
from gps_nav.gps_device import GPS

X, Y, PORT = 0, 0, 0

def loc_pub(x, y):
	global X, Y
	rospy.init_node('dgps2', anonymous=False)
	pub = rospy.Publisher('gps_pos2', latlon_gps, queue_size=10)
	try:
		X, Y = float(x)/100.0, float(y)/100.0
	except:
		X, Y = 0, 0
	pub.publish(X,Y)
	rospy.sleep(0.01)

def port_callback(data):
	global PORT
	if data.angle is not None:
		PORT = int(data.angle)

def sub_port_num():
	rospy.init_node('dgps2', anonymous=False)
	rospy.Subscriber('port_num', heading_ang, port_callback)
	rospy.sleep(0.01)


if __name__ == '__main__':
	for _ in range(10):
		sub_port_num()

	for i in range(10):
		if i == PORT:
			continue
		if i == 9:
			sys.stdout.write("\n[INFO] No Port found!\n")
			break
		try:
			serial_port = serial.Serial(
				port="/dev/ttyUSB"+str(i),
				baudrate=115200,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE
						   )
			time.sleep(1)
			if not serial_port.isOpen():
				pass
			else:
				gps = GPS(serial_port)
				sys.stdout.write("\n[INFO] Connection established at port USB"+str(i)+"\n")

			while not rospy.is_shutdown():
				ret = gps.read()
				#if ret:
				#	break
				frame = gps.parse()
				if len(list(frame.keys())) == 0:
					break
				sub_port_num()
				loc_pub(frame['dir_lat'], frame['dir_lon'])

			
		except:
			pass
