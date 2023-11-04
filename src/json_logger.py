#!/usr/bin/env python
import roslib; roslib.load_manifest('gps_nav')
import rospy, os
import sys, json, datetime
from gps_nav.msg import table1, status
from gps_nav.msg import latlon_gps

home = os.environ["HOME"]

CURR_GPS = "0, 0"
T1, T2 = dict(), dict()
T1_KEYS = ["waypoint_1","waypoint_2","waypoint_3","waypoint_4"]
T2_KEYS = ["engine", "winch", "mode", "feeder", "feed spread", "feed amount", "Current Location"]

def to_json_t1(data):
	global T1, T1_KEYS
	wp1, wp2, wp3, wp4 = data.wp1, data.wp2, data.wp3, data.wp4
		
	#json_data = json.dumps([wp1, wp2, wp3, wp4])
	#print(json_data)
	T1 = dict(zip(T1_KEYS, [wp1, wp2, wp3, wp4]))

def to_json_t2(data):
	global CURR_GPS, T2, T2_KEYS

	engine, winch, stop, mode, forward, measure, lift_up, feeder, LT, RT, F_sp, F_amt = data.engine, data.winch, data.stop, data.mode, data.forward, data.measure, data.lift_up, data.feeder, data.LT, data.RT, data.F_sp, data.F_amt

	if engine == "2":
		engine = "ON"
	else:
		engine = "OFF"

	if winch == "2":
		winch = "ON"
	else:
		winch = "OFF"
	
	if feeder == "1":
		feeder = "ON"
	else:
		feeder = "OFF"

	data = [engine, winch, mode, feeder, F_sp, F_amt, CURR_GPS]
	#print(json_data)
	T2 = dict(zip(T2_KEYS, data))

def to_strng(data):
	global CURR_GPS
	CURR_GPS = str(data.lat)+", "+str(data.lon)
	
def table1_sub():
	rospy.init_node('json_log_sub', anonymous=False)
	rospy.Subscriber("wp_table1", table1, to_json_t1)
	rospy.sleep(0.01)

def table2_sub():
	rospy.init_node('json_log_sub', anonymous=False)
	rospy.Subscriber("boat_status", status, to_json_t2)
	rospy.sleep(0.01)

def latlon_sub():
	rospy.init_node('json_log_sub', anonymous=False)
	rospy.Subscriber("gps_pos", latlon_gps, to_strng)
	rospy.sleep(0.01)

if __name__ == '__main__':
	t1_path = home+"/share/table1.json"
	t2_path = home+"/share/table2.json"
	upload_time = 0
	data1, data2 = dict(), dict()
	dt = str(datetime.datetime.now())
	dt = dt[:-10]+":00"
	prev_t = int(dt[-5:-3])
	now_t = prev_t + 5

	while not rospy.is_shutdown():
		latlon_sub()
		table1_sub()
		table2_sub()		

		dt = str(datetime.datetime.now())
		dt = dt[:-10]+":00"
		prev_t = int(dt[-5:-3])
		
		data1['time'] = [dt]
		data1['data'] = [T1]
		data2['time'] = [dt]
		data2['data'] = [T2]

		if (now_t >= prev_t + 1) or (now_t >= (prev_t-60) + 1):
			with open(t1_path, "w") as f1:
				json.dump(data1, f1)
				f1.close()
			with open(t2_path, "w") as f2:
				json.dump(data2, f2)
				f2.close()
			prev_t = now_t

		dt = str(datetime.datetime.now())
		dt = dt[:-10]+":00"
		now_t = int(dt[-5:-3])
