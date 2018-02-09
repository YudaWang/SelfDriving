# PID Project

## The goals / requirements of this project are:

* Implement and optimize a PID controller at C++ environment.

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

* Twiddle method was not directly used to find the best PID parameters since the iterative process was too slow.

* 2D (Proportional gain as one dimension and Dirivative gain as another dimension, both in log scale) scan was used to find the global minimum of total driving error (sum of cte^2) in one track. Details shown below. Only p and d gains are optimized since i gain is usually very small value.

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
