#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 10 17:13:59 2024

@author: hassan
"""

import os, sys
import  numpy as np
import matplotlib.pyplot as plt
import argparse

# # fls = os.listdir(DATA_PATH)
# # files = []
# x_l, y_l, theta_l = [], [], []

# with open(DATA_PATH) as f:
# 	lines = f.readlines()
# 	line = [l.splitlines() for l in lines]
# 	for l in line:
# 		x, y, theta = l[0].split(',')
# 		x_l.append(float(x))
# 		y_l.append(float(y))	
# 		theta_l.append(float(theta))

# # print(x_l[:5],  y_l[:5])
# mask = np.where(np.array(x_l) > 0)[0]

# # mask = list(map(int, m))
# # print(mask.shape)
# # print(mask)
# x_f, y_f, theta_f = x_l[mask[0]:mask[-1]], y_l[mask[0]:mask[-1]], theta_l[mask[0]:mask[-1]]

# y_unique = np.unique(y_f)

# x_pts, y_pts = [], []
# for y_val in y_unique:
#     if y_val in y_l:
#         y_pts.append(y_val)
#         idx = np.where(np.array(y_l) == y_val)[0][0]
#         x_pts.append(x_l[idx])

def prune_wpts(path_to_raw):
    x_l, y_l, theta_l = [], [], []
    x_pts, y_pts = [], []
    content = []
    THRES = 0.6

    with open(path_to_raw) as f:
    	lines = f.readlines()
    	line = [l.splitlines() for l in lines]
    	for l in line:
    		x, y, theta, _ = l[0].split(',')
    		x_l.append(float(x))
    		y_l.append(float(y))	
    		theta_l.append(float(theta))
    
    mask = np.where(np.array(x_l) > 0)[0]
    x_f, y_f, theta_f = x_l[mask[0]:mask[-1]], y_l[mask[0]:mask[-1]], theta_l[mask[0]:mask[-1]]
    
    if len(x_f) <= len(y_f):
        uniques = np.unique(x_f)
        for x_val in uniques:
            if x_val in x_l:
                x_pts.append(x_val)
                idx = np.where(np.array(x_l) == x_val)[0][0]
                y_pts.append(y_l[idx])

    else:
        uniques = np.unique(y_f)
        for y_val in uniques:
            if y_val in y_l:
                y_pts.append(y_val)
                idx = np.where(np.array(y_l) == y_val)[0][0]
                x_pts.append(x_l[idx])

    diff = np.abs(np.diff(y_pts))
    m = np.concatenate(([False], diff > THRES))
    
    x_pts, y_pts = np.array(x_pts)[~m].tolist(), np.array(y_pts)[~m].tolist()
    #x_pts.insert(0,x_f[0])
    #x_pts.append(x_l[-1])
    #y_pts.insert(0,y_f[0])
    #y_pts.append(y_l[-1])

    for i in range(len(x_pts)):
        content.append(str(x_pts[i])+","+str(y_pts[i])+"\n")

    return content

if __name__ == '__main__':
	parser = argparse.ArgumentParser()
	parser.add_argument("src_file_path",
		    help="Provide a valid path to the comma separated UTM data file")
	parser.add_argument("dest_file_path",
		    help="Provide a valid path to the destination folder to save pruned UTM points")
	args = parser.parse_args()
	content = prune_wpts(args.src_file_path)
	dest_file_name = args.src_file_path.split('/')[-1].split('.')[0]
	with open(args.dest_file_path+"/"+dest_file_name+"_pruned.txt", "w") as f:
		for line in content:
			f.write(line)
		f.close()
	print("\n[INFO] Pruned data logged.")


#print(len(x_pts), len(y_pts))

#plt.scatter(x_pts, y_pts)
#plt.xlabel("Easting")
#plt.ylabel("Northing")
#plt.title("Linear Motion")
#plt.show()
