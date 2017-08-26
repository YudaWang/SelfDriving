# **Behavioral Cloning**

### Yuda Wang 20170824
---
## **Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./CornerTrainingStrategy.jpeg "Corner Train Strategy"
[image2]: ./DrivingCenter.jpg "Drive on center"
[image3]: ./DrivingLeft.jpg "Drive by left"
[image4]: ./DrivingRight.jpg "Drive by right"
[image5]: ./RightTurnInward "Right turn in"
[image6]: ./RightTurnOutward.jpg "Right turn out"
[image7]: ./LeftTurnInward.jpg "Left turn in"
[image8]: ./LeftTurnOutward.jpg "Left turn out"

## Rubric Points
###Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

---
### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md or writeup_report.pdf summarizing the results
* video.mp4 contain the video of autonomous driving around track1 four times.
* CornerTrainingStrategy.jpeg contain the methodolgy of corner training data taking.

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture/pipeline has been employed (model.py lines 47~80)

My model started with image cropping and normalization to dump redundant data and improve fitting efficiency.  
Then 4 layers of convolution neural networks are introduced to extract information from images, where nonlinearity(Relu) and image size reduction(MaxPooling2D) is applied on each CNN layer.
With the concern of overfitting, dropouts are introduced after 2 CNN layers. I could have introduced more dropouts but sometime more dropouts will cause fitting converge too slow.
After flatten, 4 more layers of directly connected NN is introduced along with nonlinearity(Relu) before final decision.

Layer (type)                 Output Shape              Param #   
=================================================================
cropping2d_1 (Cropping2D)    (None, 80, 318, 3)        0         
_________________________________________________________________
lambda_1 (Lambda)            (None, 80, 318, 3)        0         
_________________________________________________________________
conv2d_1 (Conv2D)            (None, 76, 314, 24)       1824      
_________________________________________________________________
activation_1 (Activation)    (None, 76, 314, 24)       0         
_________________________________________________________________
max_pooling2d_1 (MaxPooling2 (None, 38, 157, 24)       0         
_________________________________________________________________
dropout_1 (Dropout)          (None, 38, 157, 24)       0         
_________________________________________________________________
conv2d_2 (Conv2D)            (None, 34, 153, 36)       21636     
_________________________________________________________________
activation_2 (Activation)    (None, 34, 153, 36)       0         
_________________________________________________________________
max_pooling2d_2 (MaxPooling2 (None, 17, 76, 36)        0         
_________________________________________________________________
conv2d_3 (Conv2D)            (None, 13, 72, 48)        43248     
_________________________________________________________________
activation_3 (Activation)    (None, 13, 72, 48)        0         
_________________________________________________________________
max_pooling2d_3 (MaxPooling2 (None, 6, 36, 48)         0         
_________________________________________________________________
conv2d_4 (Conv2D)            (None, 4, 34, 64)         27712     
_________________________________________________________________
activation_4 (Activation)    (None, 4, 34, 64)         0         
_________________________________________________________________
max_pooling2d_4 (MaxPooling2 (None, 2, 17, 64)         0         
_________________________________________________________________
dropout_2 (Dropout)          (None, 2, 17, 64)         0         
_________________________________________________________________
flatten_1 (Flatten)          (None, 2176)              0         
_________________________________________________________________
dense_1 (Dense)              (None, 1000)              2177000   
_________________________________________________________________
activation_5 (Activation)    (None, 1000)              0         
_________________________________________________________________
dense_2 (Dense)              (None, 100)               100100    
_________________________________________________________________
activation_6 (Activation)    (None, 100)               0         
_________________________________________________________________
dense_3 (Dense)              (None, 50)                5050      
_________________________________________________________________
activation_7 (Activation)    (None, 50)                0         
_________________________________________________________________
dense_4 (Dense)              (None, 10)                510       
_________________________________________________________________
activation_8 (Activation)    (None, 10)                0         
_________________________________________________________________
dense_5 (Dense)              (None, 1)                 11        
_________________________________________________________________
Total params: 2,377,091
Trainable params: 2,377,091
Non-trainable params: 0
_________________________________________________________________

#### 2. Decent training data acquisition strategy

My training data is well picked at different driving circumference to train the model responsive at any road conditions along the track.(No special traning data points outside of the track)

(1) I drove car in the middle of the lane for 3 tracks. During driving, even if road was straight, I didn't keep steering static but always wiggling the steering wheel in very small angle such that model can collect some knowledge about what would happen if the driving condition was non-optimal and how to respond. 
![alt text][image2]

(2) To make the model more experienced, I intentionally drove off the center(not recording) and starting collecting data and saved the car from hitting edge. I did this for 3 more tracks to give the car even more experience how to save itself in extrodinary conditions.
![alt text][image3]
![alt text][image4]

(3) Then, after brief testing, I figure out  model is already capable of handling straigh track but still can be off in the corner. In order to give it more experience in cornering, I had a combination of ways to take the corner at different circumference as shown below, so model would have full knowledge of what to do at any starting condition into the corner to bring back the car to the center of the lane after corner.
![alt text][image1]
![alt text][image5]
![alt text][image6]
![alt text][image7]
![alt text][image8]

(4) To make the cornering even better to handle extreme conditions, I did similar things as step2 but only at cornering case. So the car gain kownledge at extreme bad conditions at the corner edges.

After each of the 4 steps above I added more data to the training pool, I can see significant improvement on the driving performance the model provided. Eventually it could drive over the track smoothly at even full speed.

#### 3. Model parameter tuning

The model.py parameters are tuned to avoid either over-fitting(limited number of NN layers / dropout layers introduced) or under-fitting (still there are 4 CNN layers and 4 NN layers with decently large parameter sizes).
Also I used an adam optimizer so that manually training the learning rate wasn't necessary
Also in drive.py I optimized the speed as 12 and steering factor as 1.2 to bring better driving performance. (even the highest speed(30) is ok but I need to increase steering factors a little higher(e.g. 1.5)

