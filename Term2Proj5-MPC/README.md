# MPC Project

## The goals / requirements of this project are:

* Implement and optimize a MPC controller at C++ environment.

[//]: # (Image References)
[image1]: ./Compile.PNG
[video1]: ./P.gif
[video2]: ./PID.gif
[video3]: ./Braking.gif

---

## Project Specs

### 1. Compilation

![alt text][image1]

* C++ scripts are compilable and executable to run simulator.

### 2. Implementation

#### The Model

* The model starts with a car moving in the global coordinate with states: 

```
px: car x-position in global coordinate 
py: car y-position in global coordinate 
psi: car yaw angle in standerd math format 
v: car velocity
cte: cross track error (Position.current - Position.target)
epsi: yaw angle error (Angle.current - Angle.target)
```

* The model involves actuators:
```
delta: steering angle change
a: acceleration
```

* The model use update equations:

```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] * delta[t] * dt / Lf
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = y[t] - f(x[t]) + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] * dt / Lf

```

#### N / dt Picking Process

* N(from 1 to 100) and dt(0.001~1) has been tested and final value is optimized at N=10/dt=0.05

* When N or dt is very small, there is not enough path prediction to optimized w.r.t. waypoints given. So the current(first step) recommendation of actuator will not be the best for next several steps, which caused vehicle over-steering.

* When N or dt is very large, there will be a lot of prediction points (at the far side) well optimized to match waypoints so the algorithm is not doing very hard to compensate the prediction points that is close to car. So the current (first step) recommendation of actuator will not be best optimized as well, causing over-steering.

* So I finally picked N / dt such that at nominal speed, all prediction points are always shorter than total waypoints and usually there are not too many prediction points very far from the car that are always close to the waypoints.

|       | dt=0.001  | dt=0.01   | dt=0.1     | p=1       |
|-------|-----------|-----------|------------|-----------|
|N=1    |         |      |     |           |
|N=10   |         |         |         |           |
|N=20   |         |         |       |           |
|N=50   |      |      |        |         |
|N=100  |         |         |       |           |

* For intergral gain i, its value(i=0.0001) was optimized by a 1D scan with twiddle method when p and d values are close to optimization.

* After optimization of steering angle PID parameters, throttle value has been constructed such that it becomes a P-only function of speed, angle and cte. Its parameters are tuned such that car will quickly reach top speed if not turning and cte is small. However, whenever car is in an angle or cte is large, throttle will decrease, where negative throttle corresponding to braking.

![alt text][video3]

### 3. Reflection

* Proportional parameter works as expected: to directly correct the deviation towards the opposite direction. However, P-gain only will lead to oscillation of car in the controled dimension -- steering angle.

![alt text][video1]

* Derivative parameter works as expected: when P corrects deviation, D will reduce the amount of correction based on deviation correction rate, so overshoot/over-correct will be prevented. That is why when P=5, large value D=100 gives a very insignificant overshooting driving behavior.

![alt text][video2]

* Integral parameter is supposed to correct any long term offset as an integration effect. Its effect is not obvious in the given simulator example, which might indicates the model car offset is not significant.

### 4. Simulation

* As show in the video above, the virtual car drive thru the entire track with acceptable performance at turns with P=2, I=0.0001, D=100.
