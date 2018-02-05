# Unscented Kalman Filters Project

## The goals / requirements of this project are:

* Implement Kalman filter algorithms at C++ environment with both RADAR(polor coordinates) and LIDAR(linear coordinate) processes.
* Compile and run C++ scripts to contorl term2 simulator such that final RMSE is within expected range. 

[//]: # (Image References)
[image1]: ./UKFdata1Lidar1Radar1.png
[image2]: ./UKFdata2Lidar1Radar1.png
[image3]: ./UKFdata1Lidar1.png
[image4]: ./UKFdata2Lidar1.png
[image5]: ./UKFdata1Radar1.png
[image6]: ./UKFdata2Radar1.png
[image7]: ./GeneralProcessFlow.PNG
[image8]: ./Compile.PNG

---

## Project Specs

### 1. Compiling
![alt text][image8]
* C++ scripts are capable of being compiled by cmake and make without any error.

### 2. Accuracy

* Both dataset1 and dataset2 has been used to test UKF algorithm. Both data1 and data2 processes have been completed properly and the accuracy are within expectation.
![alt text][image1]
![alt text][image2]

### 3. General Process Flow
![alt text][image7]
The process flow is implemented based on given architecture:
* main.cpp loop import the 'measured' data and then call FusionEKF object function 'ProcessMeasurement'.
* UKF.cpp initialize all the matrices and vectors kalman filter equations need at first run, and then 'Predict' and 'Update' in linear/polar coordinate upon each loop.
* tools.cpp provides RMSE and Jacobian functions for main.cpp and FusionEKF.cpp.

### 4. Handles first measurement appropriately
* The first line of data file is used to initialize positional vector and the initial timestamp before any predict/update process starts. By this way, the first 'Update' process will use a meaningful position vector and dt won't be huge value.

### 5. Kalman filter should predict then update.
* Upon every loop, kalman filter algorithm 'predict' will always be called before 'update'.

### 6. Kalman filter can handle both RADAR and LIDAR measurements
* Both RADAR(polar coordinate) and LIDAR(linear coordinate) processes are used in FusionEKF.cpp

## Discussions

### 1. LIDAR v.s. RADAR
* UKF with LIDAR only works better than UKF with RADAR only, esp on dataset#2.
* The reason might be the fact we are using CTRV model which is more similar to linear coordinate than polar coordinate.
![alt text][image3]
![alt text][image4]
![alt text][image5]
![alt text][image6]

### 2. UKF v.s. EKF
* The best dataset1 MSRE of UKF is ~[0.06, 0.08, 0.33, 0.22], which is much better than EKF's [0.1, 0.09, 0.47, 0.47].
* The reason is probably due to EKF's much worse approximation of using Jacobian matrix to represent a matrix's derivative.

### 3. NIS
* NIS turn out to be a good indicator of how well the measurement prediction (std) is. When ~95% of the results fall in 95% NIS line, the UKF results(MRSE) is excellent.
