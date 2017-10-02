
**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./examples/car_not_car.png
[image2]: ./examples/HOG_example.jpg
[image3]: ./examples/sliding_windows.jpg
[image4]: ./examples/sliding_window.jpg
[image5]: ./examples/bboxes_and_heat.png
[image6]: ./examples/labels_map.png
[image7]: ./examples/output_bboxes.png
[video1]: ./project_video.mp4


---

### Single Frame Fitting Pipe Line

#### 1. Features: HOG, color histogram, spatial bin

The HOG feature extraction function at my script `P5-CarClassify-v5.ipnb` is named `get_hog_features()`, which takes the use of `skimage.feature` function `hog`.
The `get_hog_feature` function collects important imputs as `# pixels per cell`, `# cells per block`, `number of orientations bins in 1 cell's orientation gradient histogram`.

The color histogram feature extraction function is named `color_hist`. It imports all channels of an color image and generate histograms for each channel. One thing worth noting is the bins_range parameter need to be set as `(0,1)` if the input image intensities of each channel has been normalized to 1.

The spatial bin feature extraction function is named `bin_spatial`, which simply resize the image and flatten to a 1-D array.

For color space, I have tried RGB/HSV/HLV/... and found out full channel RGB performed the best.


#### 2. Feature Parameters

For color_space, I have tried RGB/HSV/HLV/... and found out full channel RGB performed the best.
For spatial_size and hist_bins, I have tried larger sizes and found out those wouldn't impact performance significantly.
For all HOG related parameters, I tried around and decide to use those nominal number and leave the optimization to sliding window and hot window algorithms.
For the area of interested, I excluded all pixels from y=0 to y=360, since most of the upper images are skys not road and cars.


color_space = 'RGB' # Can be RGB, HSV, LUV, HLS, YUV, YCrCb

orient = 9  # HOG orientations

pix_per_cell = 8 # HOG pixels per cell

cell_per_block = 2 # HOG cells per block

hog_channel = "ALL" # Can be 0, 1, 2, or "ALL"

spatial_size = (16, 16) # Spatial binning dimensions

hist_bins = 16    # Number of histogram bins

spatial_feat = True # Spatial features on or off


hist_feat = True # Histogram features on or off

hog_feat = True # HOG features on or off

y_start_stop = [360, 720] # Min and max in y to search in slide_window()


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
Here's a [link to my video result](./project_video.mp4)


#### 2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

