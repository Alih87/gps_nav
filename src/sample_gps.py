#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, datetime
from sbg_driver.msg import SbgGpsPos
from functools import partial

logs_path = r"/home/scout/catkin_ws/src/gps_nav/logs/"
COUNT = int(0)
def get_coords(fl, data):
    lat, long = data.latitude, data.longitude
    fl.write(str(lat)+","+str(long)+"\n")
    global COUNT
    COUNT += 1
    print(str(COUNT))

def sbg_sub(fl):
    rospy.init_node('sample_gps')
    get_coords_func = partial(get_coords, fl)
    rospy.Subscriber('/sbg/gps_pos', SbgGpsPos, get_coords_func)
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
