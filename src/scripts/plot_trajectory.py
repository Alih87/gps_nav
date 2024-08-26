#!/usr/bin/env python3
import os, sys
import  numpy as np
import matplotlib.pyplot as plt

DATA_PATH = r"/home/scout/boat_data/cont/CONT_LOG_2024_08_21_20_16:39.txt"

#fls = os.listdir(DATA_PATH)
files = []
x_l, y_l, theta_l = [], [], []

with open(DATA_PATH) as f:
	lines = f.readlines()
	line = [l.splitlines() for l in lines]
	for l in line:
		x, y, _ = l[0].split(',')
		if x=="0.0" or y=="0.0":
			pass
		else:
			x_l.append(float(x))
			y_l.append(float(y))	
			#theta_l.append(float(theta))

#print(x_l[:5],  y_l[:5])
plt.scatter(x_l, y_l[:len(x_l)])
plt.xlabel("Easting")
plt.ylabel("Northing")
plt.title("Linear Motion")
plt.show()
#plt.plot(theta_l)
#plt.xlabel("Cycles")
#plt.ylabel("Error")
#plt.title("Angle Setting")
#plt.show()

