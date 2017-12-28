#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  //measurement function - laser
  H_laser_ << 1,0,0,0,
              0,1,0,0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: 1st time initialization" << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "1st time initialization w/ Radar" << endl;/////////////////////
      float rho; float theta; float rho_dot;
      float px; float py; float vx; float vy;
      rho = measurement_pack.raw_measurements_[0]; 
      theta = measurement_pack.raw_measurements_[1];
      rho_dot = measurement_pack.raw_measurements_[2]; 
      px = rho*cos(theta);
      py = rho*sin(theta);
      vx = rho_dot*cos(theta);
      vy = rho_dot*sin(theta);
      ekf_.x_ << px, py, 0, 0;
      //ekf_.x_ << px, py, vx, vy;

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      cout << "1st time initialization w/ Lidar" << endl;//////////////////
      float px; float py;
      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
      ekf_.x_ << px, py, 0, 0;
    }

    // initialize timestamps so first dt = 0
    previous_timestamp_ = measurement_pack.timestamp_;
    // initialize P with a large value
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1, 0, 0, 0,
              0, 1, 0, 0,
              0, 0, 1000, 0,
              0, 0, 0, 1000;
    cout << "Done: 1st time initialization" << endl;////////////////////
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  //cout << "dt = " << dt << endl; ///////////////
  previous_timestamp_ = measurement_pack.timestamp_;
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ <<1,0,dt,0,
            0,1,0,dt,
            0,0,1,0,
            0,0,0,1;
  //cout << ekf_.F_ << endl; ///////////////
  float noise_ax = 6.0;
  float noise_ay = 6.0;
  float dt2 = dt*dt;
  float dt3 = dt*dt2;
  float dt4 = dt*dt3;
  ekf_.Q_ = MatrixXd(4, 4);
  ekf_.Q_ << dt4*noise_ax/4, 0, dt3*noise_ax/2, 0,
            0, dt4*noise_ay/4, 0, dt3*noise_ay/2,
            dt3*noise_ax/2, 0, dt2*noise_ax, 0,
            0, dt3*noise_ay/2, 0, dt2*noise_ay;
  //cout << ekf_.Q_ << endl; ///////////////

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;
}
