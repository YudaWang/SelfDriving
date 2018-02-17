#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

// TODO: Set the timestep length and duration
size_t N = 20;
double dt = 0.05;
unsigned idx_dt_now = int(0.1/dt);

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
