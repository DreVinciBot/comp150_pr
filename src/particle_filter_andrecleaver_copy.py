#!/usr/bin/env python

# Andre Cleaver
# COM150 PR
# 03/07/19
# HW2

import rospy
import numpy as np
import os, sys
import cv2
import random
import math
import time

class particle_filter:
    def __init__(self):
        # some fixed parameters
        self.RED = [0,0,255]
        self.BLUE = [255,0,0]
        self.ORANGE = [0,165,250]
        self.BLACK = [0,0,0]
        self.counter = 0
        self.crop_size = 50
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
        self.part_var = 100
        self.sensor_var = 5
        self.particle_array = []
        self.new_particles = []
        self.goal = False

        # load pictures by providing the path location
        img1 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/BayMap.png',1)
        img2 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/CityMap.png',1)
        img3 = cv2.imread('/home/drevinci/dre_catkin_ws/src/demo/MarioMap.png',1)

        # Obtain image properties
        px1 = img1.shape
        px2 = img2.shape
        px3 = img3.shape
        self.imgs = [img1, img2, img3]
        self.pxs = [px1, px2, px3]

        # notify if pictures are loaded.
        if not img1.any() or not img2.any() or not img3.any():
            print("Images are empty")
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

        # display coordinate of drone in traditional format
        nx = -1*int(round(yc-(yc+y)))
        ny = int(round(xc-(xc+x)))
        print("Starting position: (" + str(nx) + "," + str(ny)+")")

        # Generate the particles within a small circlular distrubuition
        self.part_dis = False
        if self.part_dis:
            rc = 150
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
            # Generate the particles in a large distribution
            for i in range(0,self.particles):
                xpt = np.random.randint(low = self.crop_size//2, high = full_px[0]- self.crop_size//2)
                ypt = np.random.randint(low = self.crop_size//2, high = full_px[1] - self.crop_size//2)
                self.particle_array.append([xpt,ypt, self.weight[i]])
            print("Number of particles: " + str(len(self.particle_array)))

        return (int(round(xc+x)), int(round(yc+y)))

    # function to crop image once given a image and x and y coordinates
    # the size of the image is set by the variable self.crop_size = 100
    def crop_image(self,f_img,x0,y0):
        cp = self.crop_size//2
        crop = f_img.copy()
        time.sleep(0.001)
        crop_img = crop[x0-cp:x0+cp,y0-cp:y0+cp]

        pixel = crop_img.shape
        row = pixel[0]
        col = pixel[1]
        ch = pixel[2]

        # adding guassian noise to the cropped image if self.noise_flag is set to True
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

    # function to display the location of the drone and the observation location
    def region_of_interest(self,wb_img,ref_x,ref_y,i):
        self.counter = self.counter + 1
        cp = self.crop_size//2
        img = wb_img.copy()

        # for visual of the drone in black
        img = cv2.circle(img, (ref_y,ref_x),10, self.BLACK, 10 )
        # Red square represents the observation image by the drone
        img = cv2.rectangle(img,(ref_y-cp,ref_x-cp),(ref_y+cp,ref_x+cp),self.RED,self.bw)

        return img

    # function to randomly move the drone a distance of 1 unit = 50 pixels;
    # this can be anagolous to the drones speed, self.speed = 50
    def movement_vector(self, x, y, i):
        x0 = x
        y0 = y
        full_px = self.pxs[i]
        cp = self.crop_size//2
        # 1 unit of distance is 50 pixels which is represented by self.speed
        theta = np.random.rand()*360
        dx = np.cos(np.deg2rad(theta))
        dy = np.sin(np.deg2rad(theta))
        # add noise to the dx and dy components with mean 0 and 5 variance
        self.dx = int(dx*self.speed + np.random.normal(0,self.sensor_var))
        self.dy = int(dy*self.speed + np.random.normal(0,self.sensor_var))

        # constraint to ensure drone moves within the image and allows to take a full
        # obsercation image of crop_size
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

        #noise_x, noise_y are coordinates for the actual drone position
        return int(noise_x), int(noise_y)

    # core of the code, takes particles and calculates the error of the particles
    # image from the observation image. I am using Root Mean Square Error (RMSE) method
    def particle_calculation(self, img, ref_image, i, xP, yP):
        score = 0
        counter = 0
        cp = self.crop_size//2
        w, h, ch = img.shape
        full_px = self.pxs[i]

        # center of image (xc,yc) used when shifting to new coordinate system. center of image is now (0,0)
        yc = self.pxs[i][1]*0.5
        xc = self.pxs[i][0]*0.5

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
                err = np.sqrt(np.mean(np.square(np.subtract(temp_img, ref_image))))
                score = math.e ** -(err)
                self.particle_array[h][2] = score

            else:
                self.particle_array[h][2] = 0

        # normalize the weights by summing the weight column and dividing each particle weight by the total weight
        self.particle_array = np.array(self.particle_array)
        v_sum = np.sum(self.particle_array[:,2])
        for h in range(len(self.particle_array)):
            pt_x, pt_y, pt_wt = self.particle_array[h][0], self.particle_array[h][1], self.particle_array[h][2]
            self.particle_array[h][2] =  pt_wt/ v_sum
            full_img = cv2.circle(img, (int(pt_y),int(pt_x)), int(self.particle_array[h][2]*1000), self.ORANGE, 3)

            # print out if goal has been met.
            if ((xP-cp) < pt_x < (xP+cp)) and ((yP-cp) < pt_y < (yP+cp)):
                counter +=1
                if counter > self.particles*0.8:
                    print("Localized on Drone!")
                    self.goal = True
        # gather all 'good' particles if the probability is greater than zero
        nice_particles = []
        for h in range(0,len(self.particle_array)):
            if self.particle_array[h][2] > 0:
                nice_particles.append(self.particle_array[h])

        # method to resample the particles based on weighted importance with replacement
        # create array of indices from the new particles
        nice_particles = np.array(nice_particles)
        idx = np.random.choice(len(nice_particles), size=self.particles, p = nice_particles[:,2])

        for h in range(self.particles):
            self.particle_array[h] = nice_particles[idx[h]]

        particle_prime = []
        for h in range(len(self.particle_array)):

            xnoise_pt = np.random.randint(low = -5,high =5)
            ynoise_pt = np.random.randint(low = -5,high =5)
            x_new = self.particle_array[h][0] + self.dx + xnoise_pt
            y_new = self.particle_array[h][1] + self.dy + ynoise_pt
            particle_prime.append([x_new, y_new, 1])
            #self.particle_array[h][2] = 1

        self.particle_array = particle_prime

        return full_img

def main():
    counter = 0
    initial_timestep = True
    pf = particle_filter()

    # Working with image 0,1, or 2
    i = 0

    # function to call a random point in the coordinate plane of image.
    # this point will serve as the drone's starting position (x0,y0)
    x0, y0 = pf.rand_location(i)

    for j in range(1,100):

        # function to move the "drone" & particles' true positino over the map
        # for each time step
        drone_x, drone_y = pf.movement_vector(x0, y0, i)

        # function to crop image given the image, crop size, and center point x0 and y0
        obs_img = pf.crop_image(pf.imgs[i],drone_x,drone_y)

        # function to display the location of the cropped image with red border
        full_img = pf.region_of_interest(pf.imgs[i],drone_x,drone_y,i)

        # Particle Filter Implementation
        test = pf.particle_calculation(full_img, obs_img, i, drone_x,drone_y )


        # display figure with particles and drone
        winname = 'TimeStep ' + str(counter)
        cv2.namedWindow(winname)        # Create a named window
        cv2.moveWindow(winname, 40,30)  # Move it to (40,30)
        cv2.imshow(winname, test)
        cv2.waitKey(100)

        if pf.goal:
            print("Localized at TimeStep : " + str(counter))
            break
        if j == 99:
            "Fail to converge..."
        #cv2.destroyAllWindows()
        x0 = drone_x
        y0 = drone_y

        counter += 1

if __name__ == '__main__':
    main()
