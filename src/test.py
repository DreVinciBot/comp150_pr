#!/usr/bin/env python
import rospy
import numpy as np
import math, operator
import cv2

from PIL import Image # No need for ImageChops
import math

image = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/BayMap.png',1)
print(image.shape)
crop_size = 100

im1 = image[0:10,0:10]


im2 = image[800:810,800:810]


err = np.sum((im1.astype("float") - im2.astype("float")) ** 2)
err /= float(im1.shape[0] * im1.shape[1])

eucliDis = np.sum((im1-im2)**2)

print(err, eucliDis)

x =  np.random.randint(crop_size//2, image.shape[0] - crop_size//2)
print(x)
