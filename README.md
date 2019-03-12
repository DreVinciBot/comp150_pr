# comp150_pr

Description of algorithm and experiment:

The python script "particle_filter_andrecleaver.py" functions as a particle filter in localizing the position of a drone within a given map using 2D images. 

Particles, which are scattered throughout the map, are treated as drones take snapshot images of the map below and are compared to the drone's reference image. 

Calculating the probability of the measurment given the position of the particle, P(z|x), is carried out by using the Root Mean Square Error (RMSE) method. After normalization this probability corresponds to the 'weight' of the particle, with a value of 1 being a perfect match. Particles that are or become out of range are assigned a weight of 0. 

Particles with weights of 0 are removed, and the remaining particles become part of a batch for resampling. Resampling is chosing particles based on weighted importance; therefore, particles with greater weights will be chosen more frequently.   

Particles and the drone then 'move' a distance of 1 unit which cooresponds to 50 pixels with added noise. 

For visualization, the full map is displayed along with the location of the drone and particles.

The code operates with a forloop that treats each iteration as a timestep, in this case dt = 1.  



To evaluate the particle filter, I will look at the average time (K,steps) it takes for the largest cluster of particles to localize within the square of drone image that represents the 'true' postion of the drone. Here the largest cluster must be 80% of the total number of particles. In my inital case 100 particles are generated and 80 of them must fall within the drone's 'line of sight'. 

I will also run the experiment comparing two different coditions. I will evaluate how the particle filter performs when changing the crop size. Here, the initial crop size is 100 pixels. I will run the same experiment but reduce the crop size by 50 pixels. By reducing the crop size I would expect the algorithm to converge at much longer timesteps and even fail to converge more frequently. 
