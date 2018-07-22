# MPC Project

## The goals / requirements of this project are:

* Implement and optimize a MPC controller at C++ environment.

[//]: # (Image References)
[image1]: ./Compile.PNG
[image2]: ./NdtLargeOverShoot.PNG
[video1]: ./Completed.gif

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

![alt text][image2]

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
```
double latency = 0.1;
double px_now = px + v*cos(psi)*latency;
double py_now = py + v*sin(psi)*latency;
```
* Then a coordinate transformation is performed to bring waypoints from global coordinate to car coordinate, using the latency corrected current car position.
```
Eigen::VectorXd ptsx_carframe(ptsx.size());
ptsx_carframe.fill(0.0);
Eigen::VectorXd ptsy_carframe(ptsy.size());
ptsy_carframe.fill(0.0);
double dx = 0;
double dy = 0;
for (unsigned iConv = 0; iConv<ptsx.size(); iConv++){
  dx = ptsx[iConv] - px_now;
  dy = ptsy[iConv] - py_now;
  ptsx_carframe[iConv] = dx*cos(psi) + dy*sin(psi);
  ptsy_carframe[iConv] = -dx*sin(psi) + dy*cos(psi);
}
```

* Finally a polynomial fitting is performed so car-frame waypoint trajectory can be described by only 3 parameters.
```
Eigen::VectorXd coeffs = polyfit(ptsx_carframe, ptsy_carframe, 2);
double cte =  -polyeval(coeffs, 0);
double epsi = -atan(coeffs[1]);
Eigen::VectorXd state(6);
state << 0,0,0, v, cte, epsi;
auto sol = mpc.Solve(state, coeffs);
```

#### MPC Processing & Final Speed Control

* The MPC processing is optimized by tuning cost function equations and weights. The most critical one is steering smoothness, where a very large weight is applied to ensure vehicle not over-shooting.
```
fg[0] = 0;
// The part of the cost based on the reference state.
for (size_t t = 0; t < N; t++) {
  fg[0] += 10*CppAD::pow(vars[cte_start + t], 2);
  fg[0] += 10*CppAD::pow(vars[epsi_start + t], 2);
  fg[0] += 1*CppAD::pow(vars[v_start + t] - ref_v, 2);
}
// Minimize the use of actuators.
for (size_t t = 0; t < N - 1; t++) {
  fg[0] += 10*CppAD::pow(vars[delta_start + t], 2);
  fg[0] += 10*CppAD::pow(vars[a_start + t], 2);
}
// Minimize the value gap between sequential actuations.
for (size_t t = 0; t < N - 2; t++) {
  fg[0] += 20000*CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
  fg[0] += 10*CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
}
```


* Finally after MPC processing, throttle value will be optimized based on predicted path points cte, epsi, psi, delta to ensure car reduce its speed when off-trajectory or wheel-turning is currently happening or will happen in the next couple of steps.
```
// define throttle control P gains
const double P_gain_v_psi = 0.5;
const double P_gain_v_cte = 0.2;
const double P_gain_v_epsi = 0.2;
const double P_gain_v_steer = 1;
double steer_value = sol.delta.at(0);
double throttle_value = sol.a.at(0);

// errors sum in the first half of predictions
double sum_psi = 0;
double sum_cte = 0;
double sum_epsi = 0;
double sum_delta = 0;
int sol_spd_red_points = int(sol.psi.size()/2);
for (int iPsi=0; iPsi<sol_spd_red_points; iPsi++){
  sum_psi += fabs(sol.psi.at(iPsi));
  sum_cte += fabs(sol.cte.at(iPsi));
  sum_epsi += fabs(sol.cte.at(iPsi));
  sum_delta += fabs(sol.delta.at(iPsi));
}

throttle_value -= P_gain_v_psi*fabs(sum_psi/sol_spd_red_points);
throttle_value -= P_gain_v_cte*fabs(sum_cte/sol_spd_red_points);
throttle_value -= P_gain_v_epsi*fabs(sum_epsi/sol_spd_red_points);
throttle_value -= P_gain_v_steer*fabs(sum_delta/sol_spd_red_points);

```

### 3. The vehicle must successfully drive a lap around the track.

* Yes.

![alt text][video1]

