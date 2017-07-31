# **Finding Lane Lines on the Road** 

## Yuda Wang 20170606

### [All video results location at  ./test_videos_output/ ]

---

**Finding Lane Lines on the Road**

The goals / steps of this project are the following:
* Make a pipeline that automatically finds lane lines on the road including challenge example
* Modify draw_lines() function so that left/right lanes can be marked by two bold lines individually, also leaving the possibilty of easy future tweaking and expansion such as multiple lanes detection or diffrent camera angles.


[//]: # (Image References)

[image1]: ./test_images_output/solidYellowLeft.jpg "PipelineEg"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

My pipeline consisted of 5 steps.
1. Turn image into gray scale
2. Use Gaussian blur and canny edge detection to extract high image contrast/gradient points into edges
3. Select region of interest so all irrelevant edges filtered
4. Transfer all edges' points within ROI to Hough space lines; extract those intersection points meeting requirements. The center points of these intersections turn out to be lines in real space.
5. Inside ROI, the left and right lanes are the two dominating groups of lines having their own similar slopes and intercepts(or theta's and rho's), along with other irrelevant lines we need to ignore. So in draw_lines() function, I transfer all lines to Hough space, they become points with two dominating aggregations zones corresponding to the left and right lanes. Then I set a range around each zone and calculate their geo-center positions in Hough Space, which finally turn out to be left/right lines in real space.

![alt text][image1]


### 2. Identify potential shortcomings with your current pipeline

1. The algorithm will becomes ineffective when location and angle of the lanes becomes obnormal
2. When there are bold edges of other road surface marks at similar angle and more intense than traffice lane lines, the current algorithm will likely be tricked to picking those meaningless structures instead of traffice lane lines.
3. Different road conditions might require different tweaks of parameters(e.g. ROI) to have its best performance.

### 3. Suggest possible improvements to your pipeline

1. In the video, each frame's lane position detection didn't use any information from any other neighbouring frames. If I can took full control of video processing, I can feed prior 2~5 frames' lane detection results into the next frame. So this moving average will reduce the next frame detection noise and thus stablize the detected lane's angle and position.
2. In order to make current algorithm feasable to more use cases, it will also be better to introduce dynamic parameters adjustments. E.g. whenever it detected more major lines than needed, the algorithm will reduce ROI or canny/Hough algorithm sensitivity.

