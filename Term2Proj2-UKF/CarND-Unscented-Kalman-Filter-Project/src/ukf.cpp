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
  x_ << 0,0,0,0,0;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);
  P_ << 1,0,0,0,0,
        0,1,0,0,0,
        0,0,1,0,0,
        0,0,0,1,0,
        0,0,0,0,1;

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 3;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 0.3;
  
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
  Xsig_pred_.fill(0.0);

  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++){
    weights_(i) = 1.0/2/(lambda_+n_aug_);
  }
  float test1= -0.123;///////////////
  cout<<test1<<endl<<abs(test1)<<endl<<fabs(test1)<<endl;//////////////////
  float test2 = -11;
  float test3 = 11;
  cout<<test2<<"\t"<<AngleNorm(test2)<<endl;
  cout<<test3<<"\t"<<AngleNorm(test3)<<endl;
  cout<<M_PI<<endl;
  cout<<atan2(999,0.001)<<endl;

  cout<<atan2(999,0.00)<<endl;

  cout<<atan2(0,0.0)<<endl;
  cout << "weights_ = "<< endl << weights_ << endl;/////////////

  ///// Count NIS3df > or < 95%
  NIS3df95up = 0;
  NIS3df95down = 0;

  ///// Count NIS2df > or < 95%
  NIS2df95up = 0;
  NIS2df95down = 0;

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
  float microstep = 0.1;

  if (is_initialized_ == false){
    cout << "UKF: 1st time initialization" << endl; /////////////////
    if(meas_package.sensor_type_ == MeasurementPackage::LASER){
      x_(0) = meas_package.raw_measurements_(0); //PositionX
      x_(1) = meas_package.raw_measurements_(1); //PositionY
    }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR){
      double rho = meas_package.raw_measurements_(0);
      double theta = meas_package.raw_measurements_(1);
      x_(0) = rho*cos(theta); //PositionX
      x_(1) = rho*sin(theta); //PositionY
    }
    time_us_ = meas_package.timestamp_;
    cout << "Done: 1st time initialization" << endl;///////////////////
    is_initialized_ = true;
  }


  dt = (meas_package.timestamp_ - time_us_) / 1000000.0;
  cout << "time interval (sec) = "<<dt<<endl;
  while (dt>=microstep){
    Prediction(microstep);
    dt -= microstep;
  }
  Prediction(dt);
  time_us_ = meas_package.timestamp_;
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_ == true ){
    UpdateLidar(meas_package);
  }else if(meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_ == true ){
    UpdateRadar(meas_package);
  }
  cout<<"================================================================"<<endl;//////////

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
  cout<<"========Start: Prediction========="<<endl;/////////////
  cout<<"x_ = " <<endl<<x_<<endl;///////
  cout<<"P_ = "<<endl<<P_<<endl;///////////
  VectorXd dx_ = VectorXd(n_x_);
  dx_.fill(0.0);
  VectorXd x_aug_ = VectorXd(n_aug_);
  x_aug_.fill(0.0);
  MatrixXd X_aug_sig_pred_ = MatrixXd(n_aug_, 2*n_aug_+1);
  X_aug_sig_pred_.fill(0.0);
  MatrixXd P_aug_sig_pred_ = MatrixXd(n_aug_, n_aug_);
  P_aug_sig_pred_.fill(0.0);
  /// Generate x,P Augmented Sigma Points
  x_aug_.head(n_x_) = x_;
  // x_aug_(3) = AngleNorm(x_aug_(3));
  P_aug_sig_pred_.topLeftCorner(n_x_, n_x_) = P_;
  P_aug_sig_pred_(n_x_, n_x_) = std_a_*std_a_;
  P_aug_sig_pred_(n_x_+1, n_x_+1) = std_yawdd_*std_yawdd_;
  MatrixXd P_aug_sig_sqrt_ = P_aug_sig_pred_.llt().matrixL();
  X_aug_sig_pred_.col(0) = x_aug_;
  for (int i=0; i<n_aug_; i++){
    X_aug_sig_pred_.col(1+i) = x_aug_ + sqrt(lambda_+n_aug_)*P_aug_sig_sqrt_.col(i);
    // X_aug_sig_pred_(3, 1+i) = AngleNorm(X_aug_sig_pred_(3, 1+i));
    X_aug_sig_pred_.col(1+n_aug_+i) = x_aug_ - sqrt(lambda_+n_aug_)*P_aug_sig_sqrt_.col(i);
    // X_aug_sig_pred_(3, 1+n_aug_+i) = AngleNorm(X_aug_sig_pred_(3, 1+n_aug_+i));
  }
  // cout<<"X_aug_sig_pred_ = "<<endl<<X_aug_sig_pred_<<endl;///////////
  /// Predict x,P Sigma Points
  for (int i=0; i<2*n_aug_+1; i++){
    VectorXd vt_pred_temp = VectorXd(n_x_);
    VectorXd att_pred_temp = VectorXd(n_x_);
    VectorXd x_temp = X_aug_sig_pred_.col(i);
    double v_k = x_temp(2);
    double psi_k = x_temp(3);
    double psi_k_dot = x_temp(4);
    double a_k = x_temp(5);
    double a_psi_k = x_temp(6);
    if (abs(psi_k_dot)<0.001){
        vt_pred_temp << v_k*cos(psi_k)*delta_t, 
                        v_k*sin(psi_k)*delta_t, 
                        0, 
                        psi_k_dot*delta_t, 
                        0;
    }else{
        vt_pred_temp << v_k/psi_k_dot*(sin(psi_k+psi_k_dot*delta_t)-sin(psi_k)),
                      v_k/psi_k_dot*(-cos(psi_k+psi_k_dot*delta_t)+cos(psi_k)),
                      0, 
                      psi_k_dot*delta_t, 
                      0;
    }
    att_pred_temp << delta_t*delta_t*cos(psi_k)*a_k/2.0, 
                      delta_t*delta_t*sin(psi_k)*a_k/2.0,
                      delta_t*a_k, 
                      delta_t*delta_t*a_psi_k/2, 
                      delta_t*a_psi_k;
    Xsig_pred_.col(i) = x_temp.head(n_x_) + vt_pred_temp + att_pred_temp;
    // Xsig_pred_(3,i) = AngleNorm(Xsig_pred_(3,i));
  }
  cout<<"Xsig_pred_ = "<<endl<<Xsig_pred_<<endl;/////////////
  /// Predict next x,P
  x_ = Xsig_pred_*weights_;
  // x_(3) = AngleNorm(x_(3));
  P_.fill(0.0);
  for(int i=0; i<2*n_aug_+1; i++){
    dx_ = Xsig_pred_.col(i) - x_;
    dx_(3) = AngleNorm(dx_(3));
    // cout<<"---> dx_ = "<<endl<<dx_<<endl;///////////
    // cout<<"---> P_ ="<<endl<<P_<<endl;///////////
    // cout<<"---> dP_ = "<<endl<<weights_(i)*dx_*dx_.transpose()<<endl;/////
    P_ += weights_(i)*dx_*dx_.transpose();
  }
  cout<<"x_ = " <<endl<<x_<<endl;///////
  cout<<"P_ = "<<endl<<P_<<endl;///////////
  cout<<"===========End: Prediction============"<<endl;
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
  cout<<"=========Start: Update Lidar==========="<<endl;//////
  cout<<"x_ = " <<endl<<x_<<endl;///////
  cout<<"P_ = "<<endl<<P_<<endl;///////////
  int n_z_ = 2;
  VectorXd dx_ = VectorXd(n_x_);
  dx_ << 0,0,0,0,0;
  VectorXd dz_ = VectorXd(n_z_);
  dz_ << 0,0;
  MatrixXd Z_aug_ = MatrixXd(n_z_, 2*n_aug_+1);
  Z_aug_.fill(0.0);
  VectorXd z_ = VectorXd(n_z_);
  z_ << 0,0;
  MatrixXd S_ = MatrixXd(n_z_,n_z_);
  S_ << 0,0,
        0,0;
  MatrixXd R_ = MatrixXd(n_z_,n_z_);
  R_ << 0,0,
        0,0;
  double nis = 0;

  ///Measurement Predict
  for (int i=0; i<2*n_aug_+1; i++){
    Z_aug_(0,i) = Xsig_pred_(0,i);
    Z_aug_(1,i) = Xsig_pred_(1,i);
  }
  z_ = Z_aug_ * weights_;
  for(int i=0; i<2*n_aug_+1; i++){
    dz_ = Z_aug_.col(i) - z_;
    S_ += weights_(i)*dz_*dz_.transpose();
  }
  R_(0,0) = std_laspx_*std_laspx_;
  R_(1,1) = std_laspy_*std_laspy_;
  S_ += R_;
  // cout << "z_ = " << endl << z_ << endl;///////////
  // cout << "S_ = " << endl << S_ << endl;////////////

  ///Measurement Update 
  MatrixXd T_ = MatrixXd(n_x_,n_z_);
  T_.fill(0.0);
  MatrixXd K_ = MatrixXd(n_x_,n_z_);
  K_.fill(0.0);
  for (int i=0; i<2*n_aug_+1; i++){
    dx_ = Xsig_pred_.col(i) - x_;
    dx_(3) = AngleNorm(dx_(3));
    dz_ = Z_aug_.col(i) - z_;
    T_ += weights_(i)*dx_*dz_.transpose();
  }
  K_ = T_*S_.inverse();
  x_ += K_*(meas_package.raw_measurements_ - z_);
  // x_(3) = AngleNorm(x_(3));
  P_ -= K_*S_*K_.transpose();
  // cout << "T_ = " << endl << T_ << endl;///////
  cout << "K_ = " << endl << K_ << endl;//////////
  cout<<"x_ = " <<endl<<x_<<endl;///////
  cout<<"P_ = "<<endl<<P_<<endl;///////////

  /// NIS
  dz_ = meas_package.raw_measurements_ - z_;
  nis = dz_.transpose()*S_.inverse()*dz_;
  cout<<"LIDAR Measurement NIS = " << nis << endl;/////////
  if (nis>5.99){
    NIS2df95up += 1;
  }else{
    NIS2df95down += 1;
  }
  cout<<"LIDAR NIS 95\% upward count = "<<NIS2df95up<<" downward count = "<<NIS2df95down<<endl;
  cout<<"==========End: Update Lidar==========="<<endl;//////
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
  cout<<"============Start: Update Radar=============="<<endl;//////
  cout<<"x_ = " <<endl<<x_<<endl;///////
  cout<<"P_ = "<<endl<<P_<<endl;///////////
  int n_z_ = 3;
  VectorXd dx_ = VectorXd(n_x_);
  dx_ << 0,0,0,0,0;
  VectorXd dz_ = VectorXd(n_z_);
  dz_ << 0,0,0;
  MatrixXd Z_aug_ = MatrixXd(n_z_, 2*n_aug_+1);
  Z_aug_.fill(0.0);
  VectorXd z_ = VectorXd(n_z_);
  z_ << 0,0,0;
  MatrixXd S_ = MatrixXd(n_z_,n_z_);
  S_ << 0,0,0,
        0,0,0,
        0,0,0;
  MatrixXd R_ = MatrixXd(n_z_,n_z_);
  R_ << 0,0,0,
        0,0,0,
        0,0,0;
  float nis = 0;
  ///Measurement Predict
  for (int i=0; i<2*n_aug_+1; i++){
    double px = Xsig_pred_(0,i);
    double py = Xsig_pred_(1,i);
    double v = Xsig_pred_(2,i);
    double psi = Xsig_pred_(3,i);
    double dpsi = Xsig_pred_(4,i);
    Z_aug_(0,i) = sqrt(px*px+py*py);
    cout<<"px= "<<px<<" py= "<<py<<" Z_aug_(0,i) = "<<Z_aug_(0,i);
    if (px!=0 || py!=0){
      Z_aug_(1,i) = atan2(py,px);
    }else{
      Z_aug_(1,i) = 0;
      cout<<"EXCEPTION1!!!"<<endl;//////////
    }
    // Z_aug_(1,i) = AngleNorm(Z_aug_(1,i));
    if (abs(Z_aug_(0,i))<0.001){
      Z_aug_(2,i) = (px*cos(psi)*v + py*sin(psi)*v)/Z_aug_(0,i);
    }else{
      Z_aug_(2,i) = 999;
      cout << "EXCEPTION2!!!" << endl; ////////////////
    }
  }
  // cout<<"Z_aug_ = "<<endl<<Z_aug_<<endl;////////////
  z_ = Z_aug_ * weights_;
  // z_(1) = AngleNorm(z_(1));
  for(int i=0; i<2*n_aug_+1; i++){
    dz_ = Z_aug_.col(i) - z_;
    dz_(1) = AngleNorm(dz_(1));
    S_ += weights_(i)*dz_*dz_.transpose();
  }
  R_(0,0) = std_radr_*std_radr_;
  R_(1,1) = std_radphi_*std_radphi_;
  R_(2,2) = std_radrd_*std_radrd_;
  S_ += R_;
  // cout<<"S_ = "<<endl<<S_<<endl;////////////

  ///Measurement Update 
  MatrixXd T_ = MatrixXd(n_x_,n_z_);
  T_.fill(0.0);
  MatrixXd K_ = MatrixXd(n_x_,n_z_);
  K_.fill(0.0);
  ////////////
  for (int i=0; i<2*n_aug_+1; i++){
    dx_ = Xsig_pred_.col(i) - x_;
    dx_(3) = AngleNorm(dx_(3));
    // cout<<"dx_ = "<<endl<<dx_<<endl;//////////
    dz_ = Z_aug_.col(i) - z_;
    dz_(1) = AngleNorm(dz_(1));
    // cout<<"dz_ = "<<endl<<dz_<<endl;///////////
    T_ += weights_(i)*dx_*dz_.transpose();
  }
  K_ = T_*S_.inverse();

  // cout<<"T_ = "<<endl<<T_<<endl;///////////
  cout<<"K_ = " <<endl<<K_<<endl;///////
  dz_ = meas_package.raw_measurements_ - z_;
  dz_(1) = AngleNorm(dz_(1));
  x_ += K_*dz_;
  P_ -= K_*S_*K_.transpose();
  cout<<"x_ = " <<endl<<x_<<endl;///////
  cout<<"P_ = "<<endl<<P_<<endl;///////////

  /// NIS
  dz_ = meas_package.raw_measurements_ - z_;
  dz_(1) = AngleNorm(dz_(1));
  nis = dz_.transpose()*S_.inverse()*dz_;
  cout<<"RADAR Measurement NIS = " << nis << endl;/////////
  if (nis>7.8){
    NIS3df95up += 1;
  }else{
    NIS3df95down += 1;
  }
  cout<<"RADAR NIS 95\% upward count = "<<NIS3df95up<<" downward count = "<<NIS3df95down<<endl;
  cout<<"============End: Update Radar==========="<<endl;//////
}

double UKF::AngleNorm(double a){
  //angle normalization
  double pi = 3.1415926;
  while (a> pi) a-=2.*pi;
  while (a<-pi) a+=2.*pi;
  return a;
}