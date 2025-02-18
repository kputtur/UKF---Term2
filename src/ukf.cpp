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
  x_ << 0.6, 0.6, 5.5, 0, 0;

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  // I experiemented with various values, finally settling to 1 here.
  // experimented values 30 - 20, 10, 5, 2, 1. 1.75
  std_a_ = 3.5/2;

  // Process noise standard deviation yaw acceleration in rad/s^2, can be tuned
  // in the range of 0.3 to 30. but I will settle for 0.5 radians
  std_yawdd_ = M_PI_2;
  
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
  DONE:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

 // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;
 
 // state dimension 
   n_x_ =  5;

 //Augumented State dimension
  n_aug_ = 7;


 // Matrix to hold sigma points
  Xsig_pred_ = MatrixXd(n_x_, 2* n_aug_ + 1);


 //Define spreading parameter
  lambda_ = 0;


//eights of sigma points
weights_ = VectorXd(2*n_aug_+1);

//start time
time_us_ = 0;

//Initialize NIS to zero
 NIS_radar_ = 0; 
 NIS_lidar_ = 0;

vis_out_file.open("obj_pose-laser-radar-ukf-output.txt", std::ofstream::out | std::ofstream::app);

}

UKF::~UKF() {
	vis_out_file.close();
}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  DONE:

  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */

  // check if it is initalized and then proceeed.

  if (!is_initialized_) {
    
    if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      double rho = meas_package.raw_measurements_(0);
      double phi = meas_package.raw_measurements_(1);
      double rhodot = meas_package.raw_measurements_(2);
      
      //convert radar from  polar to cartesian - r * cos(angle) for x and r * sin(angle) for y
      x_ << rho * cos(phi), rho * sin(phi), 4, rhodot * cos(phi), rhodot * sin(phi);
      
      //state covariance matrix
      P_ << std_radr_*std_radr_, 0, 0, 0, 0,
            0, std_radr_*std_radr_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, std_radphi_, 0,
            0, 0, 0, 0, std_radphi_;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      // Initialize state.
      x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 4, 0.5, 0.0;
      
      //state covariance matrix
      P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
            0, std_laspy_*std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    }
    
    // done initializing, no need to predict or update
    is_initialized_ = true;

    // update the time stamp
    time_us_ = meas_package.timestamp_;
    return;
    
  }
  
  // Calculate delta_t, store current time for future
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  // Predict
  Prediction(delta_t);

  vis_out_file << x_(0) << "\t";    // p1_est
  vis_out_file << x_(1) << "\t";    // p1_est
  vis_out_file << x_(2) << "\t";    // p1_est
  vis_out_file << x_(3) << "\t";    // p1_est
  vis_out_file << x_(4) << "\t";    // p1_est


  // Measurement updates
  if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    float ro = meas_package.raw_measurements_(0);
    float phi = meas_package.raw_measurements_(1);
    vis_out_file << ro * cos(phi) << "\t";    // p1_meas
    vis_out_file << ro * sin(phi) << "\t";    // p2_meas

    UpdateRadar(meas_package);
  } else {
      vis_out_file << meas_package.raw_measurements_(0) << "\t";  // p1 - meas
      vis_out_file << meas_package.raw_measurements_(1) << "\t";  // p2 - meas
      UpdateLidar(meas_package);
  }
  
}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  DONE:

  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */

 // Define spreading parameter
  //lambda_ = 3 - n_x_;
    lambda_ = max(3 - n_x_, 0);
  
  //create sigma point matrix
  MatrixXd Xsig_ = MatrixXd(n_x_, 2 * n_x_ + 1);
  
  //calculate square root of P
  MatrixXd A_ = P_.llt().matrixL();
  
  //calculate sigma points, set sigma points as columns of matrix Xsig_
  Xsig_.col(0) = x_;
  for(int i = 0; i < n_x_; i++) {
    Xsig_.col(i+1) = x_ + std::sqrt(lambda_+n_x_)*A_.col(i);
    Xsig_.col(i+1+n_x_) = x_ - std::sqrt(lambda_+n_x_)*A_.col(i);
  }
  
  // Define spreading parameter for augmentation
  lambda_ = max(3 - n_aug_, 0);
  
  //create augmented mean vector
  VectorXd x_aug_ = VectorXd(7);
  
  //create augmented state covariance
  MatrixXd P_aug_ = MatrixXd(7, 7);
  
  //create sigma point matrix
  MatrixXd Xsig_aug_ = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  
  //create augmented mean state
  x_aug_.head(5) = x_;
  x_aug_(5) = 0;
  x_aug_(6) = 0;
  
  //create augmented covariance matrix
  MatrixXd Q = MatrixXd(2,2);
  Q << std_a_*std_a_, 0,
        0, std_yawdd_*std_yawdd_;
  P_aug_.fill(0.0);
  P_aug_.topLeftCorner(5, 5) = P_;
  P_aug_.bottomRightCorner(2, 2) = Q;
  
  //create square root matrix
  MatrixXd A_aug = P_aug_.llt().matrixL();
  
  //create augmented sigma points
  Xsig_aug_.col(0) = x_aug_;
  for(int i = 0; i < n_aug_; i++) {
    Xsig_aug_.col(i+1) = x_aug_ + std::sqrt(lambda_+n_aug_)*A_aug.col(i);
    Xsig_aug_.col(i+1+n_aug_) = x_aug_ - std::sqrt(lambda_+n_aug_)*A_aug.col(i);
  }
  
  //predict sigma points
  //set vectors for each part added to x
  VectorXd vec1 = VectorXd(5);
  VectorXd vec2 = VectorXd(5);
  
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd calc_col = Xsig_aug_.col(i);
    double v = calc_col(2);
    double yaw = calc_col(3);
    double v_aug = calc_col(5);
    double v_yawdd = calc_col(6);
    double yawd = calc_col(4);
    
    //original
    VectorXd orig = calc_col.head(5);
    
    if(yawd > .001) {
      // If yaw dot is not zero
      vec1 << (v/yawd)*(sin(yaw+yawd*delta_t) - sin(yaw)),
              (v/yawd)*(-cos(yaw+yawd*delta_t) + cos(yaw)),
              0,
              yawd * delta_t,
              0;
    } else {
      // If yaw dot is zero - avoid division by zero
      vec1 << v*cos(yaw)*delta_t,
              v*sin(yaw)*delta_t,
              0,
              yawd*delta_t,
              0;
    }
    
    vec2 << .5*delta_t*delta_t*cos(yaw)*v_aug,
            .5*delta_t*delta_t*sin(yaw)*v_aug,
            delta_t*v_aug,
            .5*delta_t*delta_t*v_yawdd,
            delta_t*v_yawdd;
    
    //write predicted sigma points into right column
    Xsig_pred_.col(i) << orig + vec1 + vec2;
  }
  
  //create vector for predicted state
  VectorXd x_pred = VectorXd(n_x_);
  
  //create covariance matrix for prediction
  MatrixXd P_pred = MatrixXd(n_x_, n_x_);
  
  x_pred.fill(0.0);
  P_pred.fill(0.0);
  
  for(int i = 0; i < 2 * n_aug_ + 1; i++) {
    
    //set weights
    if (i == 0) {
      weights_(i) = lambda_ / (lambda_ + n_aug_);
    } else {
      weights_(i) = .5 / (lambda_ + n_aug_);
    }
    
    //predict state mean
    x_pred += weights_(i) * Xsig_pred_.col(i);
  }
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    
    //predict state covariance matrix
    VectorXd x_diff = Xsig_pred_.col(i) - x_pred;
    
    // angle has to be normalized 
    if (x_diff(3) > M_PI) {
      x_diff(3) -= 2. * M_PI;
    } else if (x_diff(3) < -M_PI) {
      x_diff(3) += 2. * M_PI;
    }
    P_pred += weights_(i) * x_diff * x_diff.transpose();
  }
  
  x_ = x_pred;
  P_ = P_pred;
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  DONE:

  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the lidar NIS.
  */

 //set measurement dimension, lidar can measure px and py
  int n_z = 2;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  Zsig.fill(0.0);
  z_pred.fill(0.0);
  S.fill(0.0);
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    //transform sigma points into measurement space
    VectorXd state_vec = Xsig_pred_.col(i);
    double px = state_vec(0);
    double py = state_vec(1);
    
    Zsig.col(i) << px,
                   py;
    
    //calculate mean predicted measurement
    z_pred += weights_(i) * Zsig.col(i);
  }
  
  //calculate measurement covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  
  // Add R to S
  MatrixXd R = MatrixXd(2,2);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S += R;
  
  //create vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  
  double meas_px = meas_package.raw_measurements_(0);
  double meas_py = meas_package.raw_measurements_(1);
  
  z << meas_px,
       meas_py;
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  
  //calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    
    //normalize angles
    if (x_diff(3) > M_PI) {
      x_diff(3) -= 2. * M_PI;
    } else if (x_diff(3) < -M_PI) {
      x_diff(3) += 2. * M_PI;
    }
    
    VectorXd z_diff = Zsig.col(i) - z_pred;

    Tc += weights_(i) * x_diff * z_diff.transpose();

  }
  
  // residual
  VectorXd z_diff = z - z_pred;
  
  //calculate NIS
  NIS_lidar_ = z_diff.transpose() * S.inverse() * z_diff;
  
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  x_ += K*z_diff;
  P_ -= K*S*K.transpose();

}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  DONE:

  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.

  You'll also need to calculate the radar NIS.
  */

//set measurement dimension, radar can measure r, phi, and r_dot
  int n_z = 3;
  
  //create matrix for sigma points in measurement space
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
  
  //mean predicted measurement
  VectorXd z_pred = VectorXd(n_z);
  
  //measurement covariance matrix S
  MatrixXd S = MatrixXd(n_z,n_z);
  
  Zsig.fill(0.0);
  z_pred.fill(0.0);
  S.fill(0.0);
  double rho = 0;
  double phi = 0;
  double rho_d = 0;
  
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    //transform sigma points into measurement space
    VectorXd state_vec = Xsig_pred_.col(i);
    double px = state_vec(0);
    double py = state_vec(1);
    double v = state_vec(2);
    double yaw = state_vec(3);
    //double yaw_d = state_vec(4);
    
    rho = sqrt(px*px+py*py);
    phi = atan2(py,px);
    rho_d = (px*cos(yaw)*v+py*sin(yaw)*v) / rho;
    
    Zsig.col(i) << rho,
                   phi,
                   rho_d;
    
    //calculate mean predicted measurement
    z_pred += weights_(i) * Zsig.col(i);
  }
  
  //calculate measurement covariance matrix S
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd z_diff = Zsig.col(i) - z_pred;
    if (z_diff(1) > M_PI) {
      z_diff(1) -= 2. * M_PI;
    } else if (z_diff(1) < - M_PI) {
      z_diff(1) += 2. * M_PI;
    }
    S += weights_(i) * z_diff * z_diff.transpose();
  }
  
  // Add R to S
  MatrixXd R = MatrixXd(3,3);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_;
  S += R;
  
  //create example vector for incoming radar measurement
  VectorXd z = VectorXd(n_z);
  
  double meas_rho = meas_package.raw_measurements_(0);
  double meas_phi = meas_package.raw_measurements_(1);
  double meas_rhod = meas_package.raw_measurements_(2);
  
  z << meas_rho,
       meas_phi,
       meas_rhod;
  
  //create matrix for cross correlation Tc
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  
  //calculate cross correlation matrix
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    //normalize angles
    if (x_diff(3) > M_PI) {
      x_diff(3) -= 2. * M_PI;
    } else if (x_diff(3) < -M_PI) {
      x_diff(3) += 2. * M_PI;
    }
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //normalize angles
    if (z_diff(1) > M_PI) {
      z_diff(1) -= 2. * M_PI;
    } else if (z_diff(1) < -M_PI) {
      z_diff(1) += 2. * M_PI;
    }
    Tc += weights_(i) * x_diff * z_diff.transpose();
    
  }
  
  // residual
  VectorXd z_diff = z - z_pred;
  
  //normalize angles
  if (z_diff(1) > M_PI) {
    z_diff(1) -= 2. * M_PI;
  } else if (z_diff(1) < -M_PI) {
    z_diff(1) += 2. * M_PI;
  }
  
  //calculate NIS
  NIS_radar_ = z_diff.transpose() * S.inverse() * z_diff;
  
  //calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  
  //update state mean and covariance matrix
  x_ += K*z_diff;
  P_ -= K*S*K.transpose();

}
