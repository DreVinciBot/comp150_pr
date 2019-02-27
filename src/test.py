#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2


img1 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/BayMap.png',1)
px1 = img1.shape

print(px1)

GREEN = [0,255,0]

crop_size = 2
theta = 0

var = 50

particles = 1100

l = [(0,0)]*particles

h = px1[0]
w = px1[1]

xc = px1[0]*0.5
yc = px1[1]*0.5

n_x = math.sqrt(((w/h)*particles) + ((w-h)**2)/(4.0*h**2)) - ((w-h)/(2.0*h))

n_x = np.ceil(n_x)
n_y  = np.ceil(particles//n_x)

paricles = n_x*n_y

print(paricles, n_x, n_y)

h = int(h)
w = int(w)
n_x = int(n_x)
n_y = int(n_y)

a = []
tuple(a)
image = np.zeros((h,w,3),np.uint8)

for j in range(0, h, h//n_x):
    for k in range(0, w, w//n_y):
         a.append([j,k])

for t in range(len(a)):
    xf = a[t][0]
    yf = a[t][1]

    image[xf,yf] = GREEN
    #img = cv2.circle(image,(xf,yf),1, GREEN, 1)

cv2.imshow('s_img', image)
cv2.waitKey(0)
cv2.destroyAllWindows()
