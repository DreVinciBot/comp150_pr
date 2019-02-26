#!/usr/bin/env python
import rospy
import numpy as np
import sys
import cv2
from PIL import Image
from random import random
import math

class particle_filter:
    def __init__(self):
        self.RED = [0,0,255]
        self.BLUE = [255,0,0]

        self.counter = 0
        self.crop_size = 100
        self.bw = 3

        self.move_theta = 0
        self.move_vel = 0
        self.particles = 100
        self.particle_array = [(0,0)]

        img1 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/BayMap.png',1)
        img2 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/CityMap.png',1)
        img3 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/MarioMap.png',1)

        # Obtain image properties
        px1 = img1.shape
        px2 = img2.shape
        px3 = img3.shape

        self.imgs = [img1, img2, img3]
        self.pxs = [px1, px2, px3]

        if not img1.any() or not img2.any() or not img3.any():
            print("images is empty")
        else:
            print("Images loaded. Press 0 to exit and continue.")
    # create random coordinate generator for the drone to spawn in map with appropriate range
    def rand_location(self, i):
        full_px = self.pxs[i]
        # center of image (xc,yc)
        xc = self.pxs[i][0]*0.5
        yc = self.pxs[i][1]*0.5

        x = np.random.normal(0,1)*xc
        y = np.random.normal(0,1)*yc

        # determine if the selected coordinate's corresponding cropped image is within domains
        while (xc+x)-(self.crop_size//2) < 0 or (xc+x)+(self.crop_size//2) > full_px[0]:
            x = np.random.normal(0,1)*xc
        while (yc+y)-(self.crop_size//2) < 0 or (yc+y)+(self.crop_size//2) > full_px[1]:
            y = np.random.normal(0,1)*yc

        print(-1*int(round(yc-(yc+y))), int(round(xc-(xc+x))))
        return (int(round(xc+x)), int(round(yc+y)))

        '''
        for j in range(1,self.particles+1):
            par_x = int(np.random.rand()*full_px[0])
            par_y = int(np.random.rand()*full_px[1])
            self.particle_array[j] = (par_x,par_y)
        '''
    def crop_image(self,f_img,x0,y0):
        cp = self.crop_size//2
        crop_img = f_img[x0-cp:x0+cp,y0-cp:y0+cp]
        return crop_img

    def region_of_interest(self,wb_img,x0,y0,u_x,u_y,i):
        self.counter = self.counter + 1
        print(self.counter)
        final_image = wb_img.copy()
        cp = self.crop_size//2
        img = cv2.rectangle(final_image, (y0-cp,x0-cp),(y0+cp,x0+cp), self.RED, self.bw)
        img = cv2.rectangle(img, (u_y-cp,u_x-cp),(u_y+cp,u_x+cp), self.BLUE, self.bw)
        return img

    def movement_vector(self, x, y, i):
        x0 = x
        y0 = y
        full_px = self.pxs[i]
        cp = self.crop_size//2
        # 1 unit of distance is 50 pixels
        self.move_theta = np.random.rand()*360
        dx = np.cos(np.deg2rad(theta))
        dy = np.sin(np.deg2rad(theta))
        dx = int(dx*50)
        dy = int(dy*50)
        self.move_vel = np.ceil(math.sqrt((dx)**2+(dy)**2))

        while (x0+dx)-cp < 0 or (x0+dx)+cp > full_px[0] or (y0+dy)-cp < 0 or (y0+dy)+cp > full_px[1]:
            theta = np.random.rand()*360
            print(theta)
            dx = np.cos(np.deg2rad(theta))
            dy = np.sin(np.deg2rad(theta))
            dx = int(dx*50)
            dy = int(dy*50)
            u = np.ceil(math.sqrt((dx)**2+(dy)**2))
            print("Movement vector out of bounds, redirected...")

        return self.move_theta, x0+dx, y0+dy

    def move_particles(self, par, u_x, u_y):

        p = [(0,0)]*par

        for j in range(1, len(p)):

            p[j] =
            print(p[j])






def main():
    initial_timestep = True
    paricles = 100
    pf = particle_filter()

    # image to work with [0,1,2]
    i = 1
    for j in range(1,10):
        if initial_timestep:
            # function to call a random point in the coordinate plane of image.
            x0, y0 = pf.rand_location(i)
            u_x = x0
            u_y = y0
            # function to crop image given the image, crop size, and center point x0 and y0
            rb_img = pf.crop_image(pf.imgs[i],x0,y0)
            bb_img = pf.crop_image(pf.imgs[i],u_x,u_y)
            # function to display full image with cropped region boxed
            full_img = pf.region_of_interest(pf.imgs[i],x0,y0,u_x,u_y,i)

            cv2.imshow('s_img', full_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            initial_timestep = False

        else:
            # function to move the "drone" & paricles over the map
            angle, u_x, u_y = pf.movement_vector(x0, y0, i)
            # function to crop image given the image, crop size, and center point x0 and y0
            rb_img = pf.crop_image(pf.imgs[i],x0,y0)
            bb_img = pf.crop_image(pf.imgs[i],u_x,u_y)


            # function to display the location of the cropped image with red border
            full_img = pf.region_of_interest(pf.imgs[i],x0,y0,u_x,u_y,i)

            cv2.imshow('s_img', full_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            x0 = u_x
            y0 = u_y

if __name__ == '__main__':
    main()
