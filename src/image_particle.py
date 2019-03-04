#!/usr/bin/env python
import rospy
import numpy as np
import os, sys
import cv2
from PIL import Image
from random import random
import math

class particle_filter:
    def __init__(self):
        self.RED = [0,0,255]
        self.BLUE = [255,0,0]
        self.GREEN = [0,255,0]
        self.BLACK = [0,0,0]
        self.counter = 0
        self.crop_size = 100
        self.bw = 3

        self.speed = 50
        self.dx = 0
        self.dy = 0
        self.scalar = 1000
        self.move_theta = 0
        self.move_vel = 0
        self.part_flag = True

        # Number of particles
        self.particles = 2000

        l = [1.0]*self.particles
        self.weight = np.divide(l,len(l))
        self.particle_weight = l

        # Distrubuitoin of particles
        self.part_dis = False

        # Add noise to crop images
        self.noise_flag = False

        # Concentration of particles
        self.part_var = 50
        self.sensor_var = 10
        self.particle_array = []

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

        first_step = True

        # determine if the selected coordinate's corresponding cropped image is within domains
        while (xc+x)-(self.crop_size//2) < 0 or (xc+x)+(self.crop_size//2) > full_px[0]:
            x = np.random.normal(0,1)*xc
        while (yc+y)-(self.crop_size//2) < 0 or (yc+y)+(self.crop_size//2) > full_px[1]:
            y = np.random.normal(0,1)*yc

        nx = -1*int(round(yc-(yc+y)))
        ny =  int(round(xc-(xc+x)))
        print("Starting position: (" + str(nx) + "," + str(ny)+")")

        # method of distribution of particles
        self.part_dis = False
        if self.part_dis:
            for i in range(0,self.particles):
                if first_step:
                    xpt = int(np.random.normal(xc+x,self.part_var))
                    ypt = int(np.random.normal(yc+y,self.part_var))
                    self.particle_array.append([xpt,ypt])
                    first_step = False
                else:
                    xpt = int(np.random.normal(xc+x,self.part_var))
                    ypt = int(np.random.normal(yc+y,self.part_var))
                    while [xpt,ypt] in self.particle_array:
                        xpt = int(np.random.normal(xc+x,self.part_var))
                        ypt = int(np.random.normal(yc+y,self.part_var))
                    self.particle_array.append([xpt,ypt])

        else:
            for i in range(0,self.particles):
                if first_step:
                    xpt = np.random.randint(low = self.crop_size//2, high = full_px[0]- self.crop_size//2)
                    ypt = np.random.randint(low = self.crop_size//2, high = full_px[1] - self.crop_size//2)
                    self.particle_array.append([xpt,ypt])
                    first_step = False
                else:
                    xpt = np.random.randint(low = self.crop_size//2, high = full_px[0]-self.crop_size//2)
                    ypt = np.random.randint(low = self.crop_size//2, high = full_px[1] - self.crop_size//2)

                    while [xpt,ypt] in self.particle_array:
                        xpt = np.random.randint(low = self.crop_size//2, high = full_px[0]-self.crop_size//2)
                        ypt = np.random.randint(low = self.crop_size//2, high = full_px[1] - self.crop_size//2)

                        if (xc+xpt)-(self.crop_size//2) > 0 or (xc+xpt)+(self.crop_size//2) < full_px[0]:
                            xpt = np.random.randint(low = self.crop_size//2, high = full_px[0]-self.crop_size//2)

                        elif (yc+ypt)-(self.crop_size//2) > 0 or (yc+ypt)+(self.crop_size//2) < full_px[1]:
                            ypt = np.random.randint(low = self.crop_size//2, high = full_px[1] - self.crop_size//2)
                    self.particle_array.append([xpt,ypt])

        # notify me for any duplicates
        for i in range(len(self.particle_array)):
            for i1 in range(len(self.particle_array)):
                if i != i1:
                    if self.particle_array[i] == self.particle_array[i1]:
                        print("Duplicate detected...")

        print("Number of paricles: " + str(self.particles))

        # combine particles with initial weight
        for i in range(len(self.particle_array)):
            self.particle_weight[i] = [self.particle_array[i], self.weight[i]]

        return (int(round(xc+x)), int(round(yc+y)))

    def crop_image(self,f_img,x0,y0):
        cp = self.crop_size//2
        crop_img = f_img.copy()
        crop_img = crop_img[x0-cp:x0+cp,y0-cp:y0+cp]
        pixel = crop_img.shape
        row = pixel[0]
        col = pixel[1]
        ch = pixel[2]

        # adding noise to the cropped image
        if self.noise_flag:
            low = -20
            high = 20
            guass = np.random.randint(low,high, size = (row,col, ch))
            noise = np.uint8(guass)
            noise = guass.reshape(row,col,ch)

            final_crop = crop_img + noise
            final_crop = np.clip(final_crop, 0, 255)
            crop_img = np.uint8(final_crop)

            # uncomment to display the
            '''
            cv2.imshow('With noise', final_crop)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            '''
        else:
            pass

        return crop_img

    def region_of_interest(self,wb_img,x_prime,y_prime,u_x,u_y,i):
        self.counter = self.counter + 1
        img = wb_img.copy()

        # Blue circle represents the position of the drone
        # Red circle represents the observation image
        img = cv2.circle(img,(u_y,u_x),50, self.BLUE, self.bw)
        img = cv2.circle(img,(y_prime,x_prime),50, self.RED, self.bw)

        if self.part_flag:
            self.part_flag = False
            for t in range(len(self.particle_weight)):
                xf = self.particle_weight[t][0][0]
                yf = self.particle_weight[t][0][1]
                wt = self.particle_weight[t][1]
                img = cv2.circle(img, (yf,xf), int(wt*self.scalar), self.GREEN, 2)
        else:
            pass

        return img

    def movement_vector(self, x, y, i):
        x0 = x
        y0 = y
        full_px = self.pxs[i]
        cp = self.crop_size//2
        # 1 unit of distance is 50 pixels
        theta = np.random.rand()*360
        dx = np.cos(np.deg2rad(theta))
        dy = np.sin(np.deg2rad(theta))
        self.dx = int(dx*self.speed)
        self.dy = int(dy*self.speed)

        while (x0+self.dx)-cp < 0 or (x0+self.dx)+cp > full_px[0] or (y0+self.dy)-cp < 0 or (y0+self.dy)+cp > full_px[1]:
            theta = np.random.rand()*360
            print(theta)
            dx = np.cos(np.deg2rad(theta))
            dy = np.sin(np.deg2rad(theta))
            self.dx = int(dx*self.speed)
            self.dy = int(dy*self.speed)
            print("Movement vector out of bounds, redirected...")

        # add noise for the sesor measurement
        noise_x = (x0+self.dx) + np.random.normal(0,self.sensor_var)
        noise_y = (y0+self.dy) + np.random.normal(0,self.sensor_var)

        #x0+dx, y0+dy are coordinates for the reference images
        #noise_x, noise_y are coordinates for the actual drone position
        return x0+self.dx, y0+self.dy, int(noise_x), int(noise_y)

    def particle_calculation(self, img, i):


        for h in range(len(self.particle_weight)):

            xf = self.particle_weight[h][0][0]
            yf = self.particle_weight[h][0][1]
            wt = self.particle_weight[h][1]
            full_img = cv2.circle(img, (yf,xf), int(wt*self.scalar), self.GREEN, 2)

            temp_img = self.crop_image(self.imgs[i] ,xf,yf)
            cv2.imshow('particle_img', temp_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            # recalculate the weights

            self.particle_weight[h][0][0] = xf
            self.particle_weight[h][0][1] = yf
            self.particle_weight[h][1] = wt


        return full_img

def main():
    initial_timestep = True
    pf = particle_filter()

    # image to work with [0,1,2]
    i = 0
    for j in range(1,3):
        if initial_timestep:
            # function to call a random point in the coordinate plane of image.
            # this point will serve as the drone's starting position (x0,y0)
            x0, y0 = pf.rand_location(i)
            u_x = x0
            u_y = y0
            # function to crop image given the image, crop size, and center point x0 and y0
            ref_img = pf.crop_image(pf.imgs[i],x0,y0)
            obs_img = pf.crop_image(pf.imgs[i],u_x,u_y)
            # function to display full image with cropped region boxed
            full_img = pf.region_of_interest(pf.imgs[i],x0,y0,u_x,u_y,i)

            cv2.imshow('TimeStep 0', full_img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()

            initial_timestep = False

        else:
            # function to move the "drone" & paricles' true positino over the map
            # for each time step
            ref_x, ref_y, drone_x, drone_y = pf.movement_vector(x0, y0, i)

            # function to crop image given the image, crop size, and center point x0 and y0
            ref_img = pf.crop_image(pf.imgs[i],ref_x,ref_y)
            obs_img = pf.crop_image(pf.imgs[i],drone_x,drone_y)

            # function to display the location of the cropped image with red border
            full_img = pf.region_of_interest(pf.imgs[i],ref_x,ref_y,drone_x,drone_y,i)

            # function to compare cropped image to the oberserved images



            # Particle Filter Implementation
            test = pf.particle_calculation(full_img, i)
            '''
            cv2.imshow('TimeStep ' + str(pf.counter), test)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
            '''
            x0 = ref_x
            y0 = ref_y

if __name__ == '__main__':
    main()
