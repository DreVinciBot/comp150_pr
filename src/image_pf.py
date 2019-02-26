#!/usr/bin/env python

import rospy
import numpy as np
import sys
import cv2
from matplotlib import pyplot as plt
from matplotlib import patches as patches
from PIL import Image
from random import random
import math

class particle_filter:
    def __init__(self):
        self.RED = [0,0,255]
        self.WHITE = [255,255,255]
        self.BLUE = [255,0,0]
        self.counter = 0

        self.crop_size = 100
        self.bw = 3

        self.move_theta = 0
        self.move_vel = 0

        img1 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/BayMap.png',1)
        img2 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/CityMap.png',1)
        img3 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/MarioMap.png',1)

        # Obtain image properties
        px1 = img1.shape
        px2 = img2.shape
        px3 = img3.shape

        # add white border all images
        img1 = cv2.copyMakeBorder(img1,self.bw,self.bw,self.bw,self.bw,cv2.BORDER_CONSTANT,value=self.WHITE)
        img2 = cv2.copyMakeBorder(img2,self.bw,self.bw,self.bw,self.bw,cv2.BORDER_CONSTANT,value=self.WHITE)
        img3 = cv2.copyMakeBorder(img3,self.bw,self.bw,self.bw,self.bw,cv2.BORDER_CONSTANT,value=self.WHITE)

        #temp maps
        img1_temp = img1
        img2_temp = img2
        img3_temp = img3

        self.imgs = [img1, img2, img3]
        self.pxs = [px1, px2, px3]
        self.images = [img1_temp, img2_temp, img3_temp]

        if not img1_temp.any() or not img2_temp.any() or not img3_temp.any():
            print("images is empty")
        else:
            print("Images loaded. Press 0 to exit and continue.")

    # create random coordinate generator for the drone to spawn in map with appropriate range
    def rand_location(self, i):
        full_px = self.pxs[i]

        # center of image (xc,yc)
        xc = self.pxs[i][0]*0.5
        yc = self.pxs[i][1]*0.5

        x = np.random.normal(0, 1)*xc
        y = np.random.normal(0, 1)*yc

        # determine if the selected coordinate's corresponding cropped image is within domains
        while (xc-x)-(self.crop_size//2) < self.bw or (xc-x)+(self.crop_size//2) > full_px[0]-self.bw:
            x = np.random.normal(0.0, 1.0)*xc
            print("out of x bound")

        while (yc-y)-(self.crop_size//2) < self.bw or (yc-y)+(self.crop_size//2) > full_px[1]-self.bw:
            y = np.random.normal(0.0, 1.0)*yc
            print("out of y bound")

        print(-1*int(round(yc-(yc+y))), int(round(xc-(xc+x))))

        return (int(round(xc+x)), int(round(yc+y)))

    def crop_image(self,f_img,crop_size,x0,y0, color):

        crop_img = f_img[x0-(crop_size//2):x0+(crop_size//2),y0-(crop_size//2):y0+(crop_size//2)]
        border_img = cv2.copyMakeBorder(crop_img,self.bw,self.bw,self.bw,self.bw,cv2.BORDER_CONSTANT,value=color)

        return border_img

    def region_of_interest(self,wb_img,rb_img,x0,y0,bb_img, u_x, u_y, i):
        self.counter = self.counter + 1
        final_image = wb_img

        xc = int(self.pxs[i][0]*0.5)
        yc = int(self.pxs[i][1]*0.5)

        cp = int(self.crop_size//2)


        px_rb_img = rb_img.shape
        px_bb_img = bb_img.shape

        #img = cv2.rectangle(final_image, ((xc+x0)-cp,(yc+y0)-cp), ((xc+x0)+cp,(yc+y0)+cp), (0,255,0), 2)
        img = cv2.rectangle(final_image, (x0-(px_rb_img[0]//2),y0-(px_rb_img[1]//2)), (x0+(px_rb_img[0]//2),y0+(px_rb_img[1]//2)), (0,255,0), 2)

        cv2.imshow('overlap_', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        try:
            pass
            '''
            final_image[x0-(px_rb_img[0]//2):x0+(px_rb_img[0]//2), y0-(px_rb_img[1]//2):y0+(px_rb_img[1]//2)] = rb_img
            final_image[u_x-(px_bb_img[0]//2):u_x+(px_bb_img[0]//2), u_y-(px_bb_img[1]//2):u_y+(px_bb_img[1]//2)] = bb_img

            cv2.imshow('overlap_'+ str(self.counter), final_image)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            '''

        except ValueError:
            print("value error...")

    def movement_vector(self, x, y, i):
        x0 = x
        y0 = y
        full_px = self.pxs[i]

        # 1 unit of distance is 50 pixels
        theta = np.random.rand()*360
        dx = np.cos(np.deg2rad(theta))
        dy = math.sqrt(1- dx**2)
        dx = int(dx*50)
        dy = int(dy*50)
        u = np.ceil(math.sqrt((dx)**2+(dy)**2))
        #print(theta, dy, dx, u)
        #print("New location: " + str(-1*int((yc-(y0+dy)))) + " " + str(int((xc-(x0+dx)))))

        while (x0-dx)-(self.crop_size//2) < self.bw or (x0+dx)+(self.crop_size//2) > full_px[0]-self.bw or \
              (y0-dy)-(self.crop_size//2) < self.bw or (y0-dy)+(self.crop_size//2) > full_px[1]-self.bw:

            theta = np.random.rand()*360
            dx = np.cos(np.deg2rad(theta))
            dy = math.sqrt(1- dx**2)
            dx = int(dx*50)
            dy = int(dy*50)
            print("Movement vector out of bounds, redirected...")

        return theta, x0+dx, y0+dy

def main():
    initial_timestep = True

    pf = particle_filter()

    # image to work with [0,1,2]
    i = 0

    for j in range(1,4):
        if initial_timestep:
            print("Here")
            # function to call a random point in the coordinate plane of image.
            x0, y0 = pf.rand_location(i)
            # function to crop image given the image, crop size, and center point x0 and y0
            rb_img = pf.crop_image(pf.imgs[i],pf.crop_size,x0,y0, pf.RED)
            cv2.imshow('rb_img1', rb_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            #bb_img = rb_img
            u_x = x0
            u_y = y0

            # function to crop image given the image, crop size, and center point x0 and y0
            bb_img = pf.crop_image(pf.imgs[i],pf.crop_size,u_x,u_y, pf.BLUE)
            cv2.imshow('bb_img1', bb_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # function to display the location of the cropped image with red border
            f_image = pf.region_of_interest(pf.images[i], rb_img, x0, y0, bb_img, u_x, u_y, i)

            initial_timestep = False

        else:
            print("no here")
            # function to move the "drone" & paricles over the map
            angle, u_x, u_y = pf.movement_vector(x0, y0, i)
            # function to crop image given the image, crop size, and center point x0 and y0
            rb_img2 = pf.crop_image(pf.imgs[i],pf.crop_size,u_x,u_y, pf.RED)
            cv2.imshow('rb_img2', rb_img2)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # function to crop image given the image, crop size, and center point x0 and y0
            bb_img2 = pf.crop_image(pf.imgs[i],pf.crop_size,u_x,u_y, pf.BLUE)
            cv2.imshow('bb_img2', bb_img2)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            # function to display the location of the cropped image with red border
            f_image = pf.region_of_interest(pf.images[i], rb_img2, x0, y0, bb_img2, u_x, u_y,i)

            x0 = u_x
            y0 = u_y


if __name__ == '__main__':
    main()
