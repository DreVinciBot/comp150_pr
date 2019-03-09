#!/usr/bin/env python
import rospy
import numpy as np
import os, sys
import cv2
from PIL import Image
import random
import math
import time
from skimage.measure import compare_ssim as ssim
from sklearn import preprocessing


class particle_filter:
    def __init__(self):
        # some fixed parameters
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
        self.move_theta = 0
        self.move_vel = 0
        self.part_flag = True

        # Number of particles
        self.particles = 100

        l = [1.0]*self.particles
        self.weight = l

        # Distrubuitoin of particles
        self.part_dis = True

        # Add noise to crop images
        self.noise_flag = False

        # Concentration of particles
        self.part_var = 50
        self.sensor_var = 5
        self.particle_array = []
        self.new_particles = []

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
            print("Images loaded.")

    # create random coordinate generator for the drone to spawn in map with appropriate range of image
    def rand_location(self, i):
        full_px = self.pxs[i]

        # center of image (xc,yc) used when shifting to new coordinate system. center of image is now (0,0)
        yc = self.pxs[i][1]*0.5
        xc = self.pxs[i][0]*0.5

        # generate random point for drone to spawn
        y = np.random.randint(low = (-full_px[1]//2)+ self.crop_size//2 ,high = (full_px[1]//2) - self.crop_size//2)
        x = np.random.randint(low = (-full_px[0]//2)+ self.crop_size//2 ,high = (full_px[0]//2) - self.crop_size//2)

        first_step = True

        # display coordinate of drone in traditional format
        nx = -1*int(round(yc-(yc+y)))
        ny = int(round(xc-(xc+x)))
        print("Starting position: (" + str(nx) + "," + str(ny)+")")

        # generate the particles within a small circlular Distrubuition or a large image distribution
        # radius of the circle
        #self.part_dis = False
        if self.part_dis:
            rc = 100
            for i in range(0, self.particles):
                # random angle
                alpha = 2 * math.pi * random.random()
                # random radius
                R = rc * math.sqrt(random.random())
                # calculating coordinates
                xpt = R * math.cos(alpha) + xc+x
                ypt = R * math.sin(alpha) + yc+y

                self.particle_array.append([int(xpt),int(ypt),self.weight[i]])

            print("Number of particles: " + str(len(self.particle_array)))

        else:
            for i in range(0,self.particles):

                xpt = np.random.randint(low = self.crop_size//2, high = full_px[0]- self.crop_size//2)
                ypt = np.random.randint(low = self.crop_size//2, high = full_px[1] - self.crop_size//2)

                self.particle_array.append([xpt,ypt, self.weight[i]])
            print("Number of particles: " + str(len(self.particle_array)))

        return (int(round(xc+x)), int(round(yc+y)))

    def crop_image(self,f_img,x0,y0):
        cp = self.crop_size//2
        crop = f_img.copy()
        time.sleep(0.001)
        crop_img = crop[x0-cp:x0+cp,y0-cp:y0+cp]

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

        return crop_img

    def region_of_interest(self,wb_img,ref_x,ref_y,i):
        self.counter = self.counter + 1
        cp = self.crop_size//2
        img = wb_img.copy()

        #img = cv2.circle(img,(drone_y,drone_x),50, self.BLUE, self.bw)
        img = cv2.circle(img, (ref_y,ref_x),10, self.BLACK, self.bw )

        # Red square represents the observation image by the drone
        img = cv2.rectangle(img,(ref_y-cp,ref_x-cp),(ref_y+cp,ref_x+cp),self.RED,self.bw)

        # display all initial particles
        self.part_flag = False
        if self.part_flag:
            self.part_flag = False
            for t in range(len(self.particle_array)):
                xf = self.particle_array[t][0]
                yf = self.particle_array[t][1]
                wt = self.particle_array[t][2]
                img = cv2.circle(img, (yf,xf), int(wt), self.GREEN, 2)

    #    print("init ", self.particle_array)
        return img

    def movement_vector(self, x, y, i):
        x0 = x
        y0 = y
        full_px = self.pxs[i]
        cp = self.crop_size//2
        # 1 unit of distance is 50 pixels which is represented by self.speed
        theta = np.random.rand()*360
        dx = np.cos(np.deg2rad(theta))
        dy = np.sin(np.deg2rad(theta))
        self.dx = int(dx*self.speed + np.random.normal(0,self.sensor_var))
        self.dy = int(dy*self.speed + np.random.normal(0,self.sensor_var))

        while (x0+self.dx)-cp <= 0 or (x0+self.dx)+cp >= full_px[0] or (y0+self.dy)-cp <= 0 or (y0+self.dy)+cp >= full_px[1]:
            theta = np.random.rand()*360
            dx = np.cos(np.deg2rad(theta))
            dy = np.sin(np.deg2rad(theta))
            self.dx = int((dx*self.speed)+ np.random.normal(0,self.sensor_var))
            self.dy = int((dy*self.speed)+ np.random.normal(0,self.sensor_var))
            print("Movement vector out of bounds, redirected...")

        # add noise for the sesor measurement
        noise_x = (x0+self.dx)
        noise_y = (y0+self.dy)

        #x0+dx, y0+dy are coordinates for the reference images
        #noise_x, noise_y are coordinates for the actual drone position
        return int(noise_x), int(noise_y)

    def particle_calculation(self, img, ref_image, i):
        counter = 0
        score = 0
        normalize = 0
        cp = self.crop_size//2
        v = []
        ref_row, ref_col, ref_ch = ref_image.shape
        w, h, ch = img.shape
        full_px = self.pxs[i]

        # center of image (xc,yc) used when shifting to new coordinate system. center of image is now (0,0)
        yc = self.pxs[i][1]*0.5
        xc = self.pxs[i][0]*0.5
        #print("movement ", self.particle_array)
        # calculate the Root mean square error for all particles
        for h in range(len(self.particle_array)):
            p_image = self.imgs[i].copy()
            xf = self.particle_array[h][0]
            yf = self.particle_array[h][1]

            # condition to only compare images if partile is within x and y domain; otherwise assign probability to 0
            time.sleep(0.001)
            temp_img = self.crop_image(p_image ,xf,yf)
            row, col, ch = temp_img.shape

            if row == self.crop_size and col == self.crop_size :
                counter += 1

                err = np.sqrt(np.mean(np.square(np.subtract(temp_img, ref_image))))
                score = math.e ** -(err)
                self.particle_array[h][2] = score

            else:
                self.particle_array[h][2] = 0

    #    print("good particles ", self.new_particles)

        # normalize the weights
        self.particle_array = np.array(self.particle_array)
        v_sum = np.sum(self.particle_array[:,2])
        for h in range(len(self.particle_array)):
                pt_x, pt_y, pt_wt = self.particle_array[h][0], self.particle_array[h][1], self.particle_array[h][2]
                self.particle_array[h][2] =  pt_wt/ v_sum

                #pt_y, pt_x = -pt_y + full_px[1]//2, pt_x + full_px[0]//2

                full_img = cv2.circle(img, (int(pt_y),int(pt_x)), int(1), self.GREEN, 2)

        # gather all good particles if the probability is greater than zero
        nice_particles = []
        for h in range(0,len(self.particle_array)):
            if self.particle_array[h][2] > 0:
                nice_particles.append(self.particle_array[h])

    #    print("normalize ", self.new_particles)

        # method to resample the particles based on weighted importance with replacement
        # create array of indices from the new particles
        #print(np.sum(a[:,2]))
        nice_particles = np.array(nice_particles)
        idx = np.random.choice(len(nice_particles), size=self.particles, p = nice_particles[:,2])

        for h in range(self.particles):
            self.particle_array[h] = nice_particles[idx[h]]



            #full_img = cv2.circle(img, (yf,xf), int(1), self.GREEN, 2)


        # for h in range(0, len(self.particle_array)):
        #     # move particles and add noise for each particle
        #     xf = self.particle_array[h][0]
        #     yf = self.particle_array[h][1]
        #
        #     nx = np.random.randint(5)
        #     ny = np.random.randint(5)
        #     self.particle_array[h][0] = self.particle_array[h][0] + self.dx
        #     self.particle_array[h][1] = yf + self.dy
        #     print(xf, self.dx, self.particle_array[h][0])
        #     self.particle_array[h][2] = 1
            #print("dx", self.dx, "dy", self.dy)
        # reset the weights to one


        particle_prime = []
        for h in range(len(self.particle_array)):

            xnoise_pt = np.random.randint(5)
            ynoise_pt = np.random.randint(5)
            #print(self.particle_array[h][0], self.dx)
            x_new = self.particle_array[h][0] + self.dx + xnoise_pt
            #print(self.particle_array[h][0])

            #print("y: ")
            #print(self.particle_array[h][1], self.dy)
            y_new = self.particle_array[h][1] + self.dy + ynoise_pt
            #print(self.particle_array[h][1])

            particle_prime.append([x_new, y_new, 1])
            self.particle_array[h][2] = 1

            #print(particle_prime, x_new, y_new)
            #print("weight reset ", self.particle_array)

        self.particle_array = particle_prime

        # for h in range(len(self.particle_array)):
        #     print("x: ")
        #     print(self.particle_array[h][0], self.dx)
        #     self.particle_array[h][0] = self.particle_array[h][0] + self.dx
        #     print(self.particle_array[h][0])
        #
        #     print("y: ")
        #     print(self.particle_array[h][1], self.dy)
        #     self.particle_array[h][1] = self.particle_array[h][1] + self.dy
        #     print(self.particle_array[h][1])
        #
        #     self.particle_array[h][2] = 1
        #
        #     print("weight reset ", self.particle_array)




        return full_img

def main():
    counter = 0
    initial_timestep = True
    pf = particle_filter()

    i = 0
    x0, y0 = pf.rand_location(i)
    # Working with image 0,1, or 2

    for j in range(1,100):

        # function to call a random point in the coordinate plane of image.
        # this point will serve as the drone's starting position (x0,y0)

        #
        # # function to crop image given the image, crop size, and center point x0 and y0
        # obs_img = pf.crop_image(pf.imgs[i], x0, y0)
        #
        # # function to display full image with cropped region boxed
        # full_img = pf.region_of_interest(pf.imgs[i],x0,y0,i)
        #
        # # display starting image
        # height, width = full_img.shape[:2]
        # cv2.namedWindow('jpg', cv2.WINDOW_NORMAL)
        # cv2.resizeWindow('jpg', width, height)
        # cv2.imshow('jpg', full_img)
        # cv2.waitKey(0)
        # #cv2.destroyAllWindows()



        # function to move the "drone" & particles' true positino over the map
        # for each time step
        drone_x, drone_y = pf.movement_vector(x0, y0, i)

        # function to crop image given the image, crop size, and center point x0 and y0
        obs_img = pf.crop_image(pf.imgs[i],drone_x,drone_y)

        # function to display the location of the cropped image with red border
        full_img = pf.region_of_interest(pf.imgs[i],drone_x,drone_y,i)

        # Particle Filter Implementation
        test = pf.particle_calculation(full_img, obs_img, i)

        height, width = test.shape[:2]
        cv2.namedWindow('TimeStep ' + str(counter), cv2.WINDOW_NORMAL)
        cv2.resizeWindow('TimeStep ' + str(counter), width, height)
        cv2.imshow('TimeStep ' + str(counter), test)
        cv2.waitKey(0)
        #cv2.destroyAllWindows()
        #cv2.destroyAllWindows()
        x0 = drone_x
        y0 = drone_y

        counter += 1

if __name__ == '__main__':
    main()
