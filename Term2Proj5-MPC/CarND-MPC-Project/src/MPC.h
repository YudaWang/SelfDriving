#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;


// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
const double ref_v = 50;

// TODO: Set the timestep length and duration
const size_t N = 10;
const double dt = 0.05; 
// too large N, dt cause total MPC range out of way point range, so out-of-range part get terrrible fit
// too littl N, dt cause no prediction at all



struct Solution {
	vector<double> x;
	vector<double> y;
	vector<double> psi;
	vector<double> v;
	vector<double> cte;
	vector<double> epsi;
	vector<double> delta;
	vector<double> a;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Solution Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
