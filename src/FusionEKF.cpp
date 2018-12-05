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
  MatrixXd P_ = MatrixXd(4, 4);
  MatrixXd Q_ = MatrixXd(4, 4);
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
    // cout << "Initializeing Kalman Filter" << endl;
    cout << "EKF: " << endl;
    VectorXd x_(4);
    x_ << 1, 1, 1, 1;
    previous_timestamp_ = measurement_pack.timestamp_;

    float px = measurement_pack.raw_measurements_(0);
    float py = measurement_pack.raw_measurements_(1);
    float noise_ax = 9;
    float noise_ay = 9;
    
    
    MatrixXd F_(4, 4);
    F_ << 
      1, 0, 1, 0,
      0, 1, 0, 1,
      0, 0, 1, 0,
      0, 0, 0, 1;
    F_(0, 2) = dt;
    F_(1, 3) = dt;
    
    MatrixXd P(4, 4);
    P <<
      1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

    ekf_.P_ = P;
        // cout<<"sesor:"<<measurement_pack.sensor_type_<<endl;
    float dt2 = dt * dt;
    float dt3 = dt2 * dt;
    float dt4 = dt3 * dt;

    MatrixXd Q(4, 4);
    Q << dt4 / 4 * noise_ax, 0, dt3 / 2 * noise_ax, 0,
        0, dt4 / 4 * noise_ay, 0, dt3 / 2 * noise_ay,
        dt3 / 2 * noise_ay, 0, dt2 * noise_ax, 0,
        0, dt3 / 2 * noise_ay, 0, dt2 * noise_ay;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      Hj_ << 
          1, 1, 0, 0,
          1, 1, 0, 0,
          1, 1, 1, 1;

      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float rhodot = measurement_pack.raw_measurements_(2);
     
      x_ << rho*cos(phi), rho*sin(phi), rhodot*cos(phi), rhodot*sin(phi);
      // x_ << rho * cos(phi), rho * sin(phi), 0, 0;
      // cout<<"radar R before initialize:"<<R_radar_.size()<<endl;
      ekf_.Init(x_, P, F_, Hj_, R_radar_, Q);      
      // cout<<"initialized Radar R"<<ekf_.R_<<endl;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      H_laser_ <<   
        1, 0, 0, 0,
        0, 1, 0, 0;
      x_ << px, py, 0, 0;
      // cout<<"Initializing Laser"<<endl;
      ekf_.Init(x_, P, F_, H_laser_, R_laser_, Q);
    }
    

    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout<<"initialize done!"<<endl;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/
  // cout<<"Start prediction"<<endl;
  ekf_.Predict();
  // cout<<"Prediction done"<<endl;
  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // cout<<"start update Radar"<<endl;
    Tools tools;
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    // cout<<"Radar update done"<<endl;
  } else {
    // Laser updates
    // cout<<"start update lidar"<<endl;
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    // cout<<"lidar update doen"<<endl;
  }

  // print the output
  // cout << "x_ = " << ekf_.x_ << endl;
  // cout << "P_ = " << ekf_.P_ << endl;
}
 