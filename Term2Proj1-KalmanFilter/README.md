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
```
After couple of errors(typos, misunderstandings...) debugged, the C++ scripts are capable of being compiled by cmake and make as above.
```

### 2. Accuracy: RMSE between estimations and ground-turths should be <= [0.11, 0.11, 0.52, 0.52]
![alt text][image1]
```
After correctly implemented the entire process of LIDAR-RADAR fusion kalman filters along with some parameters(noise) optimization, the RMSE between current C++ scripts estimation and ground-truth values are about [0.1, 0.1, 0.47, 0.47], which is below the expected values.
```

#### 3. Train Classifier

Linear SVM classify model is used to fit the whole traning set, which composed of ~8000 car images and ~8000 non-car images. Each image is 64x64 pixels and in .png format. The entire feature extraction, fitting costs ~30sec on a regular laptop.

#### 4. Search/Classify Cars with Sliding Windows and Sharing HOG Feature Extractions
The search funciton is named `search_cars()`.

It only call raw `hog` function once but extracts the whole hog-feature matrix of all possible blocks in the area of interest.(AOI: y=360~720)

Then, with certain window size and steps, the sub hog-feature matrces are acquired simply by scrolling windows and using corresponding window size and positions to take the sub-set of the whole hog-feature matrix. By this method, I only call raw `hog` function once and it reduced the computation time of one 720x1280 image from 25sec to 5sec, or a 5X speed-up.


#### 5. Single Frame Classifying
Finally, 3 window sizes and steps(75% overlapping) are used to search cars at different distances with different image sizes.

For smaller sizes(longer distances), I used 32x32 search window and only look for them at AOI: y = 360 to 600

For mid sizes(regular distances), I used 64x64 search window and only look for them at AOI: y = 360 to 720

For large sizes(close distances), I used 128x128 search window and only look for them at AOI: y = 360 to 720
The result is show in below.

![alt text][image1]

#### 6. Hot Window Average
However, the car detection as shown in the image above is way too sensitive and noisy for real use case.

To reduce noise and make more solid predictions, heat map is introduced in funciton `car_classify()`.
All identified objects greater than threshold is finally labeled by boxes. 

![alt text][image2]


#### 7. Lane Detection
The algorithms from `Project4` is borrowed to plot lane lines and positions.

![alt text][image3]

---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./ProjectVideoOut_v5_pass.mp4)


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.
Even with hot window, single image/frame car detection still doesn't work as well as ideal.

For this concern, I added a global variable called `heat_map_seq` to store all heat maps ever generated from one video. So instead of 1 frame hot window accumulations, I can have 10 frames sum, which reduces noise even more and gives more reliable prediction.

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?
The sub HOG matrix method saves significant amount of time, but one draw-back is we can only pick windows containing certain blocks locations. It will fail if we pick windows location and size as random.


#### 2. Future works to do
Based on the car detection box location and perspective transformation, I should be able to extract cars' locations and speed w.r.t. camera, which I didn't include in this work but will be developed later on.
