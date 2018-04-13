#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

f1 = open("./CameraTrajectory.txt")
f2= open("./Trajectory.txt")
x1 = []
y1 = []
z1 = []

x2 = []
y2 = []
z2 = []
for line1 in f1:
    if line1[0] == '#':
        continue
    data1 = line1.split()
    x1.append( float(data1[0] ) )
    y1.append( float(data1[1] ) )
    z1.append( float(data1[2] ) )
for line2 in f2:
    if line2[0] == '#':
        continue
    data2 = line2.split()
    x2.append( float(data2[0] ) )
    y2.append( float(data2[1] ) )
    z2.append( float(data2[2] ) )
ax = plt.subplot( 111, projection='3d')
#ax.plot(x1,y1,z1,'r')
ax.plot(x2,y2,z2,'b')
plt.show()
