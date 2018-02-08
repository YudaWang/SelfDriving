# PID Project

## The goals / requirements of this project are:

* Implement and optimize a PID controller at C++ environment.

[//]: # (Image References)
[image1]: ./Compile.PNG
[video1]: ./P.gif
[video2]: ./PID.gif
---

## Project Specs

### 1. Compilation
![alt text][image1]
* C++ scripts are compilable and executable to run simulator.

### 2. Implementation
* Twiddle method was not directly used to find the best PID parameters since the iterative process was too slow.
* 2D (proportional gain as one dimension and dirivative gain as another dimension, both in log scale) scan was used to find the global minimum of total driving error (sum of cte^2) in one track. Details shown as below. Only p and d gains are optimized since i gain is usually very small value.

|       | p=1     | p=2     | p=5     | p=10      |
|-------|---------|---------|---------|-----------|
|d=10   |         | 4000    |  10000  |           |
|d=20   |         |         |         |           |
|d=50   |         |         | 290     |           |
|d=100  | 1090    |  433    | 207     |    400    |
|d=200  |         |         | 215     |           |
|d=500  |         | 570     |         |           |

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

