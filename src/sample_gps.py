#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, datetime
from gps_nav.msg import latlon_gps
from functools import partial

class final_dest_node(object):
    def __init__(self):
        self.logs_path = r"/home/pi/"
        self.COUNT = int(0)

    def get_coords(self, fl, data):
        lat, long = data.lat, data.lon
        fl.write(str(lat)+","+str(long)+"\n")
        self.COUNT += 1
        #print(str(self.COUNT))

    def sbg_sub(self, fl):
        rospy.init_node('sample_gps')
        get_coords_func = partial(self.get_coords, fl)
        rospy.Subscriber('gps_pos1', latlon_gps, get_coords_func)
        rospy.sleep(0.01)

if __name__ == '__main__':
    dest_obj = final_dest_node()
    dt = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M:%S")
    with open(dest_obj.logs_path + "GPS_LOG_"+dt+".txt", "w") as f:
        rospy.loginfo("Collecting GPS data ...")
        dest_obj.sbg_sub(f)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            dest_obj.sbg_sub(f)
            rate.sleep()
        f.close()
    rospy.loginfo("Data logged.")
