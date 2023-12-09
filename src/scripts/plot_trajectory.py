#!/usr/bin/env python3
import os, sys
import  numpy as np
import matplotlib.pyplot as plt

DATA_PATH = r"/home/pi/boat_data/NAV_LOG_2023_12_07_00_28:40.txt"

fls = os.listdir(DATA_PATH)
files = []
x_l, y_l, theta_l = [], [], []

with open(DATA_PATH) as f:
	lines = f.readlines()
	line = [l.splitlines() for l in lines]
	for l in line:
		x, y, theta = l[0].split(',')
		x_l.append(float(x))
		y_l.append(float(y))	
		theta_l.append(float(theta))

print(x_l[:5],  y_l[:5])
plt.plot(x_l, y_l[:len(x_l)])
plt.xlabel("Easting")
plt.ylabel("Northing")
plt.title("Linear Motion")
plt.show()
plt.plot(theta_l)
plt.xlabel("Cycles")
plt.ylabel("Error")
plt.title("Angle Setting")
plt.show()

