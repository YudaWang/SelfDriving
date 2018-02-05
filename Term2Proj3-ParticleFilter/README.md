# Particle Filter Localization Project

## The goals / requirements of this project are:

* Implement Particle filter localization algorithms at C++ environment.

[//]: # (Image References)
[image1]: ./Term2Proj3PASS.PNG

---

## Project Specs

### 1. Accuracy
![alt text][image1]
* C++ scripts with only 10 particles are capable of passing the simulator with x_error=0.158 and y_error=0.131 and yaw_error=0.005.

### 2. Performance
* Test passed in 50sec < 100sec!

### 3. General
* Yes I am using the exact particle filters as taught in class.

## Discussions

### 1. Resampling vs Centroiding
* This is only a mind experiment. For centroiding, I mean instead of randomly re-pick all samples based on their weight as probability, each sample locations will be updated based on their weights -- where higher weight particles stay stable and smaller weight particles move closer to the centroid of all particles. (by centroid, I mean the gravitational center of all the particles based on their weight) 

|           | Pro               | Con             | Suitable                                                    |
|-----------|-------------------|-----------------|-------------------------------------------------------------|
|Resampling |quick converging   |quick diverging  | for systems with not so good measurement/mapping accuracies |
|Centroiding|slow diverging     |slow converging  | for systems with better measurement/mapping accuracies      |
