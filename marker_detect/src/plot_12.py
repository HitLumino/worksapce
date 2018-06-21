#!/usr/bin/env python
# coding=utf-8

import numpy as np
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d

f1 = open("./c.txt")
f2 = open("./Trajectory_ROS.txt")
x1 = []
y1 = []
z1= []

x2= []
y2= []
z2= []

for line in f1:
    if line[0] == '#':
        continue
    data = line.split()
    #if(-(float(data[0])- 203.987)/1000])
    x1.append( -(float(data[0])- 203.987)/1000 )
    y1.append( -(float(data[1])-44.1015)/1000 ) 
    z1.append( (float(data[2])+746.817)/1000) 

for line in f2:
    if line[0] == '#':
        continue
    data = line.split()
    x2.append( float(data[1]))
    y2.append( float(data[2])) 
    z2.append( float(data[3])) 

ax = plt.subplot( 111, projection='3d')
ax.legend();

#ax.scatter(y1,x1,z1,c='r',label='AR')
#ax.scatter(x2,y2,z2,c='b',label='Visual')

ax.plot(y1,x1,z1,c='r',label='AR')
ax.plot(x2,y2,z2,c='b',label='Visual')

ax.set_xlabel("X Axis")
ax.set_xlabel("Y Axis")
ax.set_xlabel("Z Axis")
ax.set_title("AR-marker position VS Paper's Algorithm ")
plt.show()


