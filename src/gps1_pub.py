#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial, sys
from gps_nav.msg import latlon_gps
from gps_nav.gps_device import GPS

X, Y = 0, 0

def loc_pub(x, y):
	global X, Y
	rospy.init_node('dgps1', anonymous=False)
	pub = rospy.Publisher('gps_pos1', latlon_gps, queue_size=10)
	try:
		X, Y = float(x)/100.0, float(y)/100.0
	except:
		X, Y = 0, 0
	pub.publish(X,Y)
	rospy.sleep(0.01)

if __name__ == '__main__':
	for i in range(10):
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
			if not serial_port.isOpen():
				pass
			else:
				gps = GPS(serial_port)
				sys.stdout.write("\n[INFO] Connection established at port USB"+str(i)+"\n")

			while not rospy.is_shutdown():
				ret = gps.read()
				if ret:
					break
				frame = gps.parse()
				print(frame)
				loc_pub(frame['dir_lat'], frame['dir_lon'])

			
		except:
			pass

