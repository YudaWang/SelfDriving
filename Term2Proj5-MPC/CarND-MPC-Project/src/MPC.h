#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

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
