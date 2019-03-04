#!/usr/bin/env python
import rospy
import numpy as np
import math
import cv2


img1 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/BayMap.png',1)
px1 = img1.shape
row = px1[0]
col = px1[1]
ch = px1[2]

print(img1.height)

crop_img = img1[0:10,0:10]
px1 = crop_img.shape
row = px1[0]
col = px1[1]
ch = px1[2]

cv2.imshow('TimeStep 0', crop_img)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(px1)

low = -20
high = 20

guass = np.random.randint(low,high, size = (row,col, ch))
mean = (0.0)
std = (10)
#noise = np.random.normal(mean, std, img1.shape)
#noise = np.uint8(noise)
noise = guass.reshape(row,col,ch)

guass = crop_img + noise

noisy_img_clipped = np.clip(guass, 0, 255)
noisy_img_clipped = np.uint8(noisy_img_clipped)

cv2.imshow('TimeStep 0', noisy_img_clipped)
cv2.waitKey(0)
cv2.destroyAllWindows()
