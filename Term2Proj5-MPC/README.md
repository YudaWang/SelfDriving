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
- px: car x-position in global coordinate 
- py: car y-position in global coordinate 
- psi: car yaw angle in standerd math format 
- v: car velocity
- cte: cross track error (Position.current - Position.target)
- epsi: yaw angle error (Angle.current - Angle.target)
```

* The model involves actuators:
```
- delta: steering angle change
- a: acceleration
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
*

|       | p=1     | p=2     | p=5     | p=10      |
|-------|---------|---------|---------|-----------|
|d=10   |         | 4000    |  10000  |           |
|d=20   |         |         |         |           |
|d=50   |         |         | 290     |           |
|d=100  | 1090    |  433    | 207     |    400    |
|d=200  |         |         | 215     |           |
|d=500  |         | 570     |         |           |

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
