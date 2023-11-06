#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, datetime
from gps_nav.msg import latlon_gps
from functools import partial

logs_path = r"/home/pi/"
COUNT = int(0)
def get_coords(fl, data):
    lat, long = data.lat, data.lon
    fl.write(str(lat)+","+str(long)+"\n")
    global COUNT
    COUNT += 1
    print(str(COUNT))

def sbg_sub(fl):
    rospy.init_node('sample_gps')
    get_coords_func = partial(get_coords, fl)
    rospy.Subscriber('gps_pos1', latlon_gps, get_coords_func)
    rospy.sleep(0.01)

if __name__ == '__main__':
    dt = datetime.datetime.now().strftime("%Y_%m_%d_%H_%M:%S")
    with open(logs_path + "GPS_LOG_"+dt+".txt", "w") as f:
        rospy.loginfo("Collecting GPS data ...")
        sbg_sub(f)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            sbg_sub(f)
            rate.sleep()
        f.close()
    rospy.loginfo("Data logged.")
