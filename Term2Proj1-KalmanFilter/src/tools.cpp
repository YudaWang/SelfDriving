#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    * Calculate the RMSE here.
  */
    VectorXd delta = estimations - ground_truth;
    //return sqrt(delta*delta/delta.size());
    return sqrt((delta(0)*delta(0)+delta(1)*delta(1)+delta(2)*delta(2)+delta(3)*delta(3))/4)
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
    MatrixXd Hj_ = MatrixXd(3, 4);
    float px; float py; float vx; float vy;
    //x_state >> px, py, vx, vy;
    px = x_state(0);
    py = x_state(1);
    vx = x_state(2);
    vy = x_state(3);
    float pxy = sqrt(px*px + py*py);
    float pxy2 = pxy*pxy;
    float pxy3 = pxy*pxy2;
    if (pxy==0){
      Hj_ << 0, 0, 0, 0,
            0, 0, 0, 0,
            0, 0, 0, 0;
    } else {
      Hj_ << px/pxy, py/pxy, 0, 0,
            -py/pxy2, px/pxy2, 0, 0,
            py*(vx*py-vy*px)/pxy3, px*(vy*px-vx*py)/pxy3, px/pxy, py/pxy;
    }
    return Hj_;
}
