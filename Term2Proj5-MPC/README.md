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
|N=1    |    bad     |      |          |      bad     |
|N=10   |         |  good    |  good    |           |
|N=20   |         |         |       |           |
|N=50   |      |      |        |         |
|N=100  |  bad    |         |       |  bad     |


#### Model Predictive Control with Latency &  Polynomial Fitting and MPC Preprocessing

* After data fetched from simulator, latency was compromised by calculating the current car global location based on its position/speed received and amount of latency.

* Then a coordinate transformation is performed to bring waypoints from global coordinate to car coordinate, using the latency corrected current car position.

* Finally a polynomial fitting is performed so car-frame waypoint trajectory can be described by only 3 parameters.

![alt text][video3]

#### MPC Processing & Final Speed Control

* The MPC processing is optimized by tuning cost function equations and weights. The most critical one is steering smoothness, where a very large weight is applied to ensure vehicle not over-shooting.

* Finally after MPC processing, throttle value will be optimized based on predicted path points cte, epsi, psi, delta to ensure car reduce its speed when off-trajectory or wheel-turning is currently happening or will happen in the next couple of steps.


### 3. The vehicle must successfully drive a lap around the track.

* Yes.

![alt text][video1]

