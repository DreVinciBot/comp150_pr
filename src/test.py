#!/usr/bin/env python
import rospy
import numpy as np
import math

theta = 0
particles = 100

l = [(0,0)]*particles

full_px = [400, 600]

for j in range(1, len(l)):

    x = int(np.random.rand()*full_px[0])
    y = int(np.random.rand()*full_px[1])
    l[j] = (x,y)
    print(l[j])
