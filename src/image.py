#!/usr/bin/env python

import rospy
import numpy as np
import sys
import cv2
from matplotlib import pyplot as plt
from random import random
import math

RED = [0,0,255]
WHITE = [255,255,255]

crop_size = 100
bw = 3
x0 = 0
y0 = 0

img1 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/BayMap.png',1)
img2 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/CityMap.png',1)
img3 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/MarioMap.png',1)

# Obtain image properties
px1 = img1.shape
px2 = img2.shape
px3 = img3.shape

# add white border all images 
img1 = cv2.copyMakeBorder(img1,bw,bw,bw,bw,cv2.BORDER_CONSTANT,value=WHITE)
img2 = cv2.copyMakeBorder(img2,bw,bw,bw,bw,cv2.BORDER_CONSTANT,value=WHITE)
img3 = cv2.copyMakeBorder(img3,bw,bw,bw,bw,cv2.BORDER_CONSTANT,value=WHITE)

#temp maps
img1_temp = img1
img2_temp = img2
img3_temp = img3

i = 0
imgs = [img1, img2, img3]
pxs = [px1, px2, px3]
images = [img1_temp, img2_temp, img3_temp]

if not img1_temp.any() or not img2_temp.any() or not img3_temp.any():
    print("images is empty")
else:
    print("Images loaded. Press 0 to exit and continue.")

'''
cv2.imshow("Selected image", imgs[i])
cv2.waitKey(0)
cv2.destroyAllWindows()
'''


# create random coordinate generator for the drone to spawn in map with appropriate range
def rand_location():
    x = np.int(random()*pxs[i][0])
    y = np.int(random()*pxs[i][1])
    
    return x,y

def crop_image(f_img,crop_size,x0,y0):

    crop_img = f_img[x0-(crop_size//2):x0+(crop_size//2),y0-(crop_size//2):y0+(crop_size//2)]
    cp_px = crop_img.shape

    if (cp_px[0] == cp_px[1]):
        border_img = cv2.copyMakeBorder(crop_img,bw,bw,bw,bw,cv2.BORDER_CONSTANT,value=RED)
        return border_img

    else:
        x0, y0 = rand_location()
        crop_image(img,crop_size,x0,y0)

def region_of_interest(b_img,x0,y0):


    px_b_img = b_img.shape

    if (x0-(px_b_img[0]//2)> 0) and (x0+(px_b_img[0]//2)+1) < px3[0]: 
        if(y0-(px_b_img[1]//2)> 0) and (y0+(px_b_img[1]//2)+1) < px3[1]:
         
            try:
                images[i][x0-(px_b_img[0]//2):x0+(px_b_img[0]//2), y0-(px_b_img[1]//2):y0+(px_b_img[1]//2)] = b_img
                cv2.imshow('overlap', images[i])
                cv2.waitKey(0)
                cv2.destroyAllWindows()
            
            except ValueError:
                print("value error...")
                x0, y0 = rand_location()
                img = crop_image(imgs[i],crop_size,x0,y0)
                region_of_interest(img, x0,y0)

        else:
            print('Out of y bounds...')
            x0, y0 = rand_location()
            img = crop_image(imgs[i],crop_size,x0,y0)
            region_of_interest(img,x0,y0)
          
    else:
        print('Out of x bounds...')
        # function to choose next random movement vector0
        x0, y0 = rand_location()
        img = crop_image(imgs[i],crop_size,x0,y0)
        region_of_interest(img,x0,y0)

for j in range(1,30):
    x0, y0 = rand_location()
    print("Location: (" + str(x0)+ ", " + str(y0)+ ")")
    print(j)
    img = crop_image(imgs[i],crop_size,x0,y0)

    if img != None:
        print("pass")
        region_of_interest(img,x0,y0)
    else:
        print("here")
        x0, y0 = rand_location()
        img = crop_image(imgs[i],crop_size,x0,y0)
        region_of_interest(img,x0,y0)

sys.exit()
