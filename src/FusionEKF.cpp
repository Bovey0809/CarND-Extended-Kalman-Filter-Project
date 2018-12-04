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
  noise_ax = 9;
  noise_ay = 9;

  MatrixXd P(4, 4);
  P <<
    1, 0, 0, 0,
    0, 1, 0, 0,
    0, 0, 1000, 0,
    0, 0, 0, 1000;

  MatrixXd F(4, 4);
  F<<
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1, 0,
      0, 0, 0, 1;
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
    cout << "Initializeing Kalman Filter" << endl;
    cout << "EKF: " << endl;
    x_ = VectorXd(4);
    px = measurement_pack.raw_measurements_(0);
    py = measurement_pack.raw_measurements_(1);
    previous_timestamp_ = measurement_pack.timestamp_;
    float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;

    F_ <<
      1, 0, dt, 0,
      0, 1, 0, dt,
      0, 0, 1, 0,
      0, 0, 0, 1;

    flaot dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;
  
    Q_ <<
      dt4/4*noise_ax, 0,              dt3/2*noise_ax, 0,
      0,              dt4/4*noise_ay, 0,              dt3/2*noise_ay,
      dt3/2*noise_ay, 0,              dt2*noise_ax,   0,
      0,              dt3/2*noise_ay, 0,              dt2*noise_ay;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_(1);
      float phi = measurement_pack.raw_measurements_(2);
      float rhodot = measurement_pack.raw_measurements_(3);
     
      x_ << rho*cos(x), rho*sin(x), rhodot*cos(x), rhodot*sin(x);
      Hj_ = Tools::CalculateJacobian(x_);	
      ekf_.Init(x_, P, F, Hj_, R_radar_, Q_);      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      x_ << px, py, 0, 0;
      ekf_.Init(x_, P, F, H_laser_, R_laser_, Q_);
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  
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
    ekf_.Update(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
