#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2


img1 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/BayMap.png',1)
px1 = img1.shape

print(px1)

GREEN = [0,255,0]

crop_size = 100
theta = 0

var = 50

particles = 100

l = [(0,0)]*particles

h = px1[0] - crop_size
w = px1[1] - crop_size

h = 30.0
w = 20.0

xc = px1[0]*0.5
yc = px1[1]*0.5

n_x = math.sqrt(((w/h)*particles) + ((w-h)**2)/(4.0*h**2)) - ((w-h)/(2.0*h))

n_x = np.ceil(n_x)
n_y = np.ceil(particles/n_x)

paricles = n_x*n_y

spacing = np.floor(w/(n_x - 1))

#print(paricles, n_x, n_y, spacing)

a = []
tuple(a)

flag = 0

first = True
for i in range(0,particles+1):
    if first:
        x = int(np.random.normal(xc,var))
        y = int(np.random.normal(yc,var))
        a.append([x,y])
        first = False
    else:
        x = int(np.random.normal(xc,var))
        y = int(np.random.normal(yc,var))
        while [x,y] in a:
            x = int(np.random.normal(xc,var))
            y = int(np.random.normal(yc,var))
            print("Duplicated detected..." + str(i))

        a.append([x,y])

for i in range(len(a)):
        for i1 in range(len(a)):
            if i != i1:
                if a[i] == a[i1]:
                    flag = 1

print(flag)


for t in range(len(a)):
    xf = a[t][0]
    yf = a[t][1]
    img1 = cv2.rectangle(img1, (yf-1,xf-1),(yf+1,xf+1), GREEN, 2)

cv2.imshow('s_img', img1)
cv2.waitKey(0)
cv2.destroyAllWindows()
