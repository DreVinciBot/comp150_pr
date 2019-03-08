#!/usr/bin/env python
import rospy
import numpy as np
import math, operator
import cv2
from skimage.measure import compare_ssim as ssim
from matplotlib.pyplot import imshow
import matplotlib.pyplot as plt
import argparse
import imutils
import ImageChops

from random import random
from PIL import Image # No need for ImageChops
import math

from skimage import img_as_float
from skimage.measure import compare_mse as mse

image = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/MarioMap.png',1)

'''
height, width = img1.shape[:2]
cv2.namedWindow('jpg', cv2.WINDOW_NORMAL)
cv2.resizeWindow('jpg', width, height)
cv2.imshow('jpg', img1)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''



MyList = [1, 2, 3, 'a', 'b,', 'c']

idx = np.random.choice(4, size=200)




text = [[4, 3, 8, 9, 5, 1, 2, 7, 6], [8, 3, 4, 1, 5, 9, 6, 7, 2],
[6, 1, 8, 7, 5, 3, 2, 9, 4], [6, 9, 8, 7, 5, 3, 2, 1, 4],
[6, 1, 8, 7, 5, 3, 2, 1, 4], [6, 1, 3, 2, 9, 4, 8, 7, 5]]

print(text)
