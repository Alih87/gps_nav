#!/usr/bin/env python3
import os, sys
import  numpy as np
import matplotlib.pyplot as plt

DATA_PATH = "/home/pi/utm/"

fls = os.listdir(DATA_PATH)
files = []
x_l, y_l, theta_l = [], [], []
for i in fls:
    #if len(i) > 31 and "SUCCESS" in i:
	files.append(i)

for f in files:
	x_l, y_l, theta_l = [], [], []
	with open(DATA_PATH+f) as f:
		lines = f.readlines()
		line = [l.splitlines() for l in lines]
		for l in line:
			x, y, theta = l[0].split(',')
			x_l.append(float(x))
			y_l.append(float(y))	
			#theta_l.append(float(theta))
			if len(x_l) > 999999:
				break
	#print(x_l[:5],  y_l[:5])
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

