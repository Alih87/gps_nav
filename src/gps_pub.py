#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, serial
from gps_nav.msg import latlon_gps
from gps_nav.gps_device import GPS

def loc_pub(X, Y):
	rospy.init_node('gps_raw', anonymous=False)
	pub = rospy.Publisher('gps_pos', latlon_gps, queue_size=10)
	try:
		X, Y = float(X), float(Y)
	except:
		X, Y = 0, 0
	pub.publish(X,Y)
	rospy.sleep(0.01)

if __name__ == '__main__':
	for i in range(5):
		if i == 5:
			print("\n[INFO] No Port found!\n")
			break
		try:
			serial_port = serial.Serial(
				port="/dev/ttyUSB"+str(i),
				baudrate=115200,
				bytesize=serial.EIGHTBITS,
				parity=serial.PARITY_NONE,
				stopbits=serial.STOPBITS_ONE
						   )

			gps = GPS(serial_port)

			print("\n[INFO] Connection established at port USB"+str(i))
			
			while not rospy.is_shutdown():
				gps.read()
				frame = gps.parse()
				loc_pub(frame['lat'], frame['lon'])
		except:
			pass


