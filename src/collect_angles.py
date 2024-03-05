#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys, datetime
from sbg_driver.msg import SbgMag, SbgEkfEuler
from geometry_msgs.msg import Vector3Stamped
from math import atan, atan2, pi

class Collector(object):
	def __init__(self, device, collect):
		self.device = device
		self.collect = collect
		self.angle = 0
		self.xy, self.yy, self.zy = [], [], []
		self.xm, self.ym, self.zm = [], [], []
		self.yaw_angles, self.mag_angles = [], []
		if self.device == 'sbg':
			if self.collect == 'mag':
				self.msg_type = SbgMag
				self.topic = '/sbg/mag'
				self.callback = self.sbg_mag_cb
			elif self.collect == 'pose':
				self.topic = '/sbg/ekf_euler'
				self.callback = self.sbg_pose_cb
			else:
				raise Exception("Invalid collection choice.")
		elif self.device == 'um7':
			self.msg_type = Vector3Stamped
			if self.collect == 'mag':
				self.topic = '/imu/mag'
				self.callback = self.um7_mag_cb
			elif self.collect == 'pose':
				self.topic = '/imu/rpy'
				self.callback = self.um7_pose_cb
			else:
				raise Exception("Invalid collection choice.")
		else:
			raise Exception("Unrecognized Device.")

	def um7_mag_cb(self, data):
		x, y, z = data.vector.x, data.vector.y, data.vector.z
		self.xm.append(x)
		self.ym.append(y)
		self.zm.append(z)

	def um7_pose_cb(self, data):
		x, y, z = data.vector.x, data.vector.y, data.vector.z
		self.xy.append(x)
		self.yy.append(y)
		self.zy.append(z)

	def sbg_mag_cb(self, data):
		x, y = data.mag.x, data.mag.y
		angle = atan2(y,x)*(180/pi)
		self.mag_angles.append(angle)

	def sbg_pose_cb(self, data):
		x, y, z = data.angle.x, data.angle.y, data.angle.z
		self.yaw_angles.append(z*(180/pi))

	def scout_pose_cb(self, data):
		z = data.pose.pose.orientation.z
		w = data.pose.pose.orientation.w
		angle = atan2(2.0*(w*z), 1.0-2.0*(z*z))*(180/pi)

	def collect_mag(self):
		rospy.init_node('collect_angle', anonymous=False)
		rospy.Subscriber('/imu/mag', Vector3Stamped, self.um7_mag_cb)

	def collect_yaw(self):
		rospy.init_node('collect_angle', anonymous=False)
		rospy.Subscriber('/imu/rpy', Vector3Stamped, self.um7_pose_cb)

if __name__ == '__main__':
	coll = Collector(device='sbg', collect='pose')
	while not rospy.is_shutdown():
		coll.collect_mag()
		coll.collect_yaw()

	with open("/home/scout/boat_data/mag_collect_raw.txt", 'w') as f2:
		for i,j,k in zip(coll.xm, coll.ym, coll.zm):
			f2.write(str(i)+","+str(j)+","+str(k)+"\n")
		f2.close()
