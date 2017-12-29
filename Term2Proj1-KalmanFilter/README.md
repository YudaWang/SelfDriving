# Extended Kalman Filters Project

## The goals / requirements of this project are:

* Configure Windows10 Ubuntu-BASH environment and set up uWebSocketIO interface between running C++ scripts and term2 simulator.
* Implement Kalman filter algorithms at C++ environment with both RADAR(polor coordinates) and LIDAR(linear coordinate) processes.
* Compile and run C++ scripts to contorl term2 simulator such that final RMSE is within expected range. 

[//]: # (Image References)
[image1]: ./Dataset1EKFtracking.PNG
[image2]: ./Dataset2EKFtracking.PNG
[image3]: ./Dataset1EKFtracking-LidarOnly.PNG
[image4]: ./Dataset1EKFtracking-RadarOnly.PNG
[image5]: ./MeetSpec-compiled.PNG
[image6]: ./MeetSpec-ProcessFlow.PNG
[image7]: ./MeetSpec-1stMeasHandle.PNG

---

## Project Specs

### 1. Compiling: Code must compile without errors with cmake and make.
![alt text][image5]
#### After couple of errors(e.g. ms-sec conversion factor set as 100000 instead of 1000000 / misunderstood on RMSE algorithms / not end C++ line with ';' / C++ class functions can only be called within object not directly from class / y need to be normalized ) debugged, the C++ scripts are capable of being compiled by cmake and make as above.

### 2. Accuracy: RMSE between estimations and ground-turths should be <= [0.11, 0.11, 0.52, 0.52]
![alt text][image1]
#### After correctly implemented the entire process of LIDAR-RADAR fusion kalman filters along with some parameters(process noise) optimization, the RMSE between current C++ scripts estimation and ground-truth values are about [0.1, 0.1, 0.47, 0.47], which is below the expected values.

### 3. Correct Algo: Process flow as taught in the preceding lessons
![alt text][image6]
#### The process flow is implemented based on the given architecture:
* main.cpp loop import the 'measured' data and then call FusionEKF object function 'ProcessMeasurement'.
* FusionEKF.cpp initialize all the matrices and vectors kalman filter equations would need and then call EKF functions 'Predict' and 'Update' or 'UpdateEKF'.
* kalman_filter.cpp implements 'Predict' function under linear cooridinate and 'Update' function under both linear and polar coordinates. 
* tools.cpp provides RMSE and Jacobian functions for main.cpp and FusionEKF.cpp.

### 4. Correct Algo: Handles first measurement appropriately
![alt text][image7]
#### The first line of data file is used to initialize positional vector and the initial timestamp before any predict/update process starts.

### 5. Correct Algo: Kalman filter should predict then update.
#### Upon every loop, kalman filter algorithm 'predict' will always be called before 'update'.

### 6. Correct Algo: Kalman filter can handle both RADAR and LIDAR measurements
#### Both RADAR(polar coordinate) and LIDAR(linear coordinate) processes are used in FusionEKF.cpp

### 7. Extra: RADAR v.s. LIDAR measurement responses
### LIDAR only
![alt text][image3]

### RADAR only
![alt text][image4]



---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?
The sub HOG matrix method saves significant amount of time, but one draw-back is we can only pick windows containing certain blocks locations. It will fail if we pick windows location and size as random.


#### 2. Future works to do
Based on the car detection box location and perspective transformation, I should be able to extract cars' locations and speed w.r.t. camera, which I didn't include in this work but will be developed later on.
