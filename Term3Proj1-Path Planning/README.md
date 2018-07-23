# Path Planning Project

## The goals / requirements of this project are:

* Design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic.

[//]: # (Image References)
[image1]: ./4p77MilesPassing.PNG
[video1]: ./1lap16x.gif

---

## Project Specs

### 1. Compilation

* C++ scripts are compilable and executable to run simulator.

### 2. Valid Trajectories

![alt text][image1]

* The car is able to drive at least 4.32 miles without incident.

* The car drives according to the speed limit.

* Max Acceleration and Jerk are not Exceeded.

* Car does not have collisions.

* The car stays in its lane, except for the time between changing lanes.

* The car is able to change lanes.


![alt text][video1]


## Reflection / Documentation

### Path finding control process:

* Motion sensing: 

- The host vehicle's current position, speed and planned future path waypoints can be acquired by its own sensing feedback. 

--All other vehicle's current location and speed can be acquired by sensor fusion information.

* Speed control: The car top speed has been capped at 49.5mps to ensure it never exceed 50mps limit. On different cases: if there is no car in front within 30m of host vehicle's 1sec future location,

* Lane control:

* Path generation and execution:
