#include "kalman_filter.h"
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;
// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  std::cout<<"z_pred:"<<z_pred.size()<<std::endl;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  

  // VectorXd zp = H_ * x_;
  // VectorXd y = z - zp;
  // MatrixXd Ht = H_.transpose();
  // MatrixXd S = H_ * P_ * Ht + R_;
  // MatrixXd K = P_ * Ht * S.inverse();

  // // New estimate
  // x_ = x_ + K * y;
  // long x_size = x_.size();
  // MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  float rho, phi, rhodot;
  rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
  // The phi should be normalized to -pi and pi.
  phi = atan2(x_(1), x_(0));
  //cout<<"RHO:"<<rho<<"PHI:"<<phi<<endl; No problem.
  if (fabs(rho) < 0.0001) {
    rhodot = 0;
  }
  
  else
  {
    rhodot = (x_(0) * x_(2) + x_(1) * x_(3)) / rho;
  }
  // cout<<"rho dot:"<<rhodot<<endl;
  VectorXd z_pred(3);
  z_pred << rho, phi, rhodot;
  // cout<<"z prediction:"<<z_pred<<endl;
  VectorXd y = z - z_pred;
  // cout<<"y:"<<y<<endl;
  MatrixXd Ht = H_.transpose();
  // cout<<"H_:"<<H_.size()<<endl<<"Ht:"<<Ht.size()<<endl;
  // cout<<"P_.shape:"<<P_.size()<<endl;
  // cout<<"R_.shape"<<R_.size()<<endl;

  MatrixXd S = H_ * P_ * Ht + R_;
  // cout<<"S:"<<S<<endl;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  // cout<<"K:"<<K<<endl;
  //new estimate
  // cout << "K x y:" << K*y<<endl;
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  // cout<<"x_: "<<x_<<endl;
  // cout<<"P_: "<<P_<<endl;
  // VectorXd y = z - z_pred;
  // MatrixXd Ht = H_.transpose();
  // MatrixXd S = H_ * P_ * Ht + R_;
  // MatrixXd K = P_ * Ht * S.inverse();

  // // New estimate
  // x_ = x_ + K * y;
  // long x_size = x_.size();
  // MatrixXd I = MatrixXd::Identity(x_size, x_size);
  // P_ = (I - K * H_) * P_;
}