#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from math import atan, atan2, pi
import tf, math

class model_node():
    def __init__(self, model_path):
        self.model_path = model_path

    def instantiate(self):
        pass

if __name__ == '__main__':
	pass
