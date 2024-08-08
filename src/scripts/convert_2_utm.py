#!/usr/bin/env python3
from utm import from_latlon
import sys
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("src_path",
                    help="Provide a valid path to the comma separated latitude and longitude data file")
parser.add_argument("dest_path",
                    help="Provide a valid path to save the comma separated utm coordinates file")
args = parser.parse_args()

CENTER = (388731.70, 3974424.49)

def get_utm(src, dst):
	with open(src) as fr:
		raw_pts = fr.readlines()
	with open(dst, "w") as f:
		for line in raw_pts:
		    lat_str, long_str = line.split(",")
		    X, Y, zo, ne = from_latlon(float(lat_str), float(long_str))
		    X, Y = X - CENTER[0], Y - CENTER[1]
		    ZONE = str(zo)+ne
		    f.write(str(X)+","+str(Y)+","+str(ZONE)+"\n")
		f.close()

if __name__ == "__main__":
    sys.stdout.write("[INFO] Converting Lat and Long to UTM ...\n")
    get_utm(args.src_path, args.dest_path)
    sys.stdout.write("[INFO] Done and saved to file.")
