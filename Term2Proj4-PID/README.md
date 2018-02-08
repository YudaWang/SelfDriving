# PID Project

## The goals / requirements of this project are:

* Implement and optimize a PID controller at C++ environment.

[//]: # (Image References)
[image1]: ./Compile.PNG
[video1]: ./P.gif
[video2]: ./PID.gif

---

## Project Specs

### 1. Compilation
![alt text][image1]
* C++ scripts are compilable and executable to run simulator.

### 2. Implementation
* Twiddle method was not directly used to find the best PID parameters since the iterative process was too slow.
* 2D (proportional gain as one dimension and dirivative gain as another dimension, both in log scale) scan was used to find the global minimum of total driving error (sum of cte^2) in one track. Details shown as below. Only p and d gains are optimized since i gain is usually very small value.

|       | p=1     | p=2     | p=5     | p=10      |
|-------|---------|---------|---------|-----------|
|d=10   |         | 4000    |  10000  |           |
|d=20   |         |         |         |           |
|d=50   |         |         | 290     |           |
|d=100  | 1090    |  433    | 207     |    400    |
|d=200  |         |         | 215     |           |
|d=500  |         | 570     |         |           |

* For intergral gain i, its value(i=0.005) was optimized by a 1D scan with twiddle method after p and d values are close to optimization.

### 3. Reflection
* P parameter works as expected: to directly correct the deviation towards the opposite direction. However, only P will lead to oscillation of car in the controled dimension -- steering.

![alt text][video1]

* D parameter works as expected: when P corrects deviation, D will reduce the amount of correction based on deviation correction rate. So overshoot/over-correct will be prevented. That is why when P=5, D=100 gives a very small overshooting driving behavior.

![alt text][video2]

* I parameter is supposed to correct any long term offset as an integration effect. Its effect is not obvious in the given simulator example, which might indicates the model car offset is not significant.

### 4. Simulation
* As show in the video above, the virtual car drive thru the entire track with acceptable performance at turns.
