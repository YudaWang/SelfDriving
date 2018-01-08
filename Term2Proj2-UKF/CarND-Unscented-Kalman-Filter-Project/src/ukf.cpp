#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {


  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.
  
  /**
  TODO: Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off..
  */
  // at 1st run, need initialization
  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3-n_x_;
  Xsig_pred_ = MatrixXd(n_x_, 2*n_aug_+1);
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+2*n_aug_+1);
  for (int i=1; i<2*n_aug_+1; i++){
    weights_(i) = 1.0/2/(lambda_+2*n_aug_+1);
  }
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
  float dt = 0.0;

  if (is_initialized_ == false){
    cout << "UKF: 1st time initialization" << endl; /////////////////
    if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      x_(0) = sensor_type_.raw_measurements_(0); //PositionX
      x_(1) = sensor_type_.raw_measurements_(1); //PositionY
    }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      float rho = sensor_type_.raw_measurements_(0);
      float theta = sensor_type_.raw_measurements_(1);
      x_(0) = rho*cos(theta); //PositionX
      x_(1) = rho*sin(theta); //PositionY
    }
    time_us_ = sensor_type_.timestamp_;
    for (int i=0; i<5; i++){
      P_(i,i) = 100; //Covariences
    } 
    cout << "Done: 1st time initialization" << endl;///////////////////
    is_initialized_ = true;
  }

  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == ture){
    dt = (meas_package.timestamp_ - time_us_) * 1000000;
    time_us_ = meas_package.timestamp_;
    Prediction(dt);
    UpdateLidar(meas_package);
  }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == ture){
    dt = (meas_package.timestamp_ - time_us_) * 1000000;
    time_us_ = meas_package.timestamp_;
    Prediction(dt);
    UpdateRadar(meas_package);
  }

}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */
  VectorXd dx_ = VectorXd(n_x_);
  VectorXd x_aug = VectorXd(n_aug_);
  MatrixXd X_aug_sig_pred_ = MatrixXd(n_aug_, 2*n_aug_+1);
  MatrixXd P_aug_sig_pred_ = MatrixXd(n_aug_, n_aug_);
  /// Generate x,P Sigma Points
  x_aug.head(n_x_) = x_;
  x_aug[n_x_] = 0;
  x_aug[n_x_+1] = 0;
  P_aug_sig_pred_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_sig_pred_(n_x_, n_x_) = std_a_*std_a_;
  P_aug_sig_pred_(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;
  MatrixXd P_aug_sig_sqrt_ = P_aug_sig_pred_.llt().matrixL();
  X_aug_sig_pred_.col(0) = x_aug;
  for (int i=0; i<n_aug_; i++){
    X_aug_sig_pred_.col(1+i) = x_ + sqrt(lambda_+n_aug_)*P_aug_sig_sqrt_.col(i);
    X_aug_sig_pred_.col(1+n_aug_+i) = x_ - sqrt(lambda_+n_aug_)*P_aug_sig_sqrt_.col(i);
  }
  /// Predict x,P Sigma Points
  for (int i=0; i<2*n_aug_+1; i++){
    VectorXd vt_pred_temp = VectorXd(n_x_);
    VectorXd att_pred_temp = VectorXd(n_x_);
    VectorXd x_temp = X_aug_sig_pred_.col(i);
    float v_k = x_temp(2);
    float psi_k = x_temp(3);
    float psi_k_dot = x_temp(4);
    float a_k = x_temp(5);
    float a_psi_k = x_temp(6);
    if (psi_k_dot==0){
        vt_pred_temp << v_k*cos(psi_k)*delta_t, 
                        v_k*sin(psi_k)*delta_t, 
                        0, 
                        psi_k_dot*delta_t, 
                        0;
        att_pred_temp << delta_t*delta_t*cos(psi_k)*a_k/2, 
                        delta_t*delta_t*sin(psi_k)*a_k/2, 
                        delta_t*a_k, 
                        delta_t*delta_t*a_psi_k/2, 
                        delta_t*a_psi_k;  
    }else{
        vt_pred_temp << v_k/psi_k_dot*(sin(psi_k+psi_k_dot*delta_t)-sin(psi_k)),
                      v_k/psi_k_dot*(-cos(psi_k+psi_k_dot*delta_t)+cos(psi_k)),
                      0, 
                      psi_k_dot*delta_t, 
                      0;
        att_pred_temp << delta_t*delta_t*cos(psi_k)*a_k/2, 
                      delta_t*delta_t*sin(psi_k)*a_k/2,
                      delta_t*a_k, 
                      delta_t*delta_t*a_psi_k/2, 
                      delta_t*a_psi_k;
    }
    Xsig_pred_.col(i) = x_temp.head(n_x_) + vt_pred_temp + att_pred_temp;
  }
  /// Predict next x,P
  x_ = Xsig_pred_*weights_;
  P_ = MatrixXd(n_x_, n_x_);
  for(int i=0; i<2*n_aug_+1; i++){
    dx_ = Xsig_pred_.col(i) - x_;
    P_ += weights_(i)*dx_*dx_.transpose();
  }
}



/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */
  VectorXd dx_ = VectorXd(n_x_);
  float nis = 0;
  ///Measurement Predict
  int n_z_ = 2;
  MatrixXd Z_aug_ = MatrixXd(n_z_, 2*n_aug_+1);
  VectorXd z_ = VectorXd(n_z_);
  MatrixXd S_ = MatrixXd(n_z_,n_z_);
  for (int i=0; i<2*n_aug_+1; i++){
    Z_aug_(0,i) = Xsig_pred_(0,i);
    Z_aug_(1,i) = Xsig_pred_(1,i);
  }
  z_ = Z_aug_ * weights_;
  VectorXd dz_ = VectorXd(n_z_);
  for(int i=0; i<2*n_aug_+1; i++){
    dz_ = Z_aug_.col(i) - z_;
    S_ += weights_*dz_*dz_.transpose();
  }
  MatrixXd R_ = MatrixXd(n_z_,n_z_);
  R_(0,0) = std_laspx_*std_laspx_;
  R_(1,1) = std_laspy_*std_laspy_;
  S_ += R_;

  ///Measurement Update 
  MatrixXd T_ = MatrixXd(n_x_,n_z_);
  MatrixXd K_ = MatrixXd(n_x_,n_z_);
  for (int i=0; i<2*n_aug_+1; i++){
    dx_ = Xsig_pred_.col(i) - x_;
    dz_ = Z_aug_.col(i) - z_;
    T_ += weights_(i)*dx_*dz_.transpose();
  }
  K_ = T_*S_.inverse();
  x_ += K_*(meas_package.raw_measurements_ - z_);
  P_ -= K_*S_*K_.transpose();

  /// NIS
  dz_ = meas_package.raw_measurements_ - z_;
  nis = dz_.transpose()*S_.inverse()*dz_;
}


/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */
  VectorXd dx_ = VectorXd(n_x_);
  float nis = 0;
  ///Measurement Predict
  int n_z_ = 3;
  MatrixXd Z_aug_ = MatrixXd(n_z_, 2*n_aug_+1);
  VectorXd z_ = VectorXd(n_z_);
  MatrixXd S_ = MatrixXd(n_z_,n_z_);
  for (int i=0; i<2*n_aug_+1; i++){
    float px = Xsig_pred_(0,i);
    float py = Xsig_pred_(1,i);
    float v = Xsig_pred_(2,i);
    float psi = Xsig_pred_(3,i);
    float dpsi = Xsig_pred_(4,i);
    Z_aug_(0,i) = sqrt(px*px+py*py);
    Z_aug_(1,i) = atan2(py,px);
    Z_aug_(2,i) = (px*cos(psi)*v + py*sin(psi)*v)/Zsig(0,i);
  }
  z_ = Z_aug_ * weights_;
  VectorXd dz_ = VectorXd(n_z_);
  for(int i=0; i<2*n_aug_+1; i++){
    dz_ = Z_aug_.col(i) - z_;
    S_ += weights_*dz_*dz_.transpose();
  }
  MatrixXd R_ = MatrixXd(n_z_,n_z_);
  R_(0,0) = std_radr_*std_radr_;
  R_(1,1) = std_radphi_*std_radphi_;
  R_(2,2) = std_radrd_*std_radrd_;
  S_ += R_;

  ///Measurement Update 
  MatrixXd T_ = MatrixXd(n_x_,n_z_);
  MatrixXd K_ = MatrixXd(n_x_,n_z_);
  for (int i=0; i<2*n_aug_+1; i++){
    dx_ = Xsig_pred_.col(i) - x_;
    dz_ = Z_aug_.col(i) - z_;
    T_ += weights_(i)*dx_*dz_.transpose();
  }
  K_ = T_*S_.inverse();
  x_ += K_*(meas_package.raw_measurements_ - z_);
  P_ -= K_*S_*K_.transpose();

  /// NIS
  dz_ = meas_package.raw_measurements_ - z_;
  nis = dz_.transpose()*S_.inverse()*dz_;
}
