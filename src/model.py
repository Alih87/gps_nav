#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, sys
from math import atan, atan2, pi
from gps_nav.GI_NN import GI_NN

class model_node():
    def __init__(self, input_size, output_channels, seq_len, chkpts_path):
        self.chkpts_path = chkpts_path
        # self.model = GI_NN(input_size=input_size, output_channels=output_channels, seq_len=seq_len)
        # self.model = self.model.float()
        # self.model.train()
        

    def single_training_iter(self):
        pass

if __name__ == '__main__':
    chkpts = "/home/scout/catkin_ws/gps_nav/src/chkpts"
    