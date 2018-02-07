# PID Project

## The goals / requirements of this project are:

* Implement and optimize a PID controller at C++ environment.

[//]: # (Image References)
[image1]: ./Compile.PNG
[video1]: ./ScreenCaptureProject2_1.gif

---

## Project Specs

### 1. Compilation
![alt text][image1]
* C++ scripts with only 10 particles are capable of passing the simulator with x_error=0.158 and y_error=0.131 and yaw_error=0.005.
![alt text][video1]
### 2. Performance
* Test passed in 50sec < 100sec!

### 3. General
* Yes I am using the exact particle filters as taught in class.

## Discussions

### 1. Resampling vs Centroiding
* This is only a mind experiment. And it seems to me the wheel resampling process is effective for general case but when measurement/mapping is very good, my 'centroiding' method might provide less noisy control.

|           | Methodology                                             |
|-----------|---------------------------------------------------------|
|Resampling |randomly re-pick all samples based on their weight as probability|
|Centroiding|each sample updated based on its weight, where higher weight particles stay stable and smaller weight particles move closer to centroid of all particles|
* Centroid: weight/gravitational center of all particles

|           | Pro            | Con           | Suitable                                                    |
|-----------|----------------|---------------|-------------------------------------------------------------|
|Resampling |quick converging|quick diverging| for systems with generally mediacre measurement/mapping accuracies with lots of outliers|
|Centroiding|slow diverging  |slow converging| for systems with generally better measurement/mapping accuracies but few outliers|
