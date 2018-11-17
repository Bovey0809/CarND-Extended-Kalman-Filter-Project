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
  VectorXd RMSE(4);
  RMSE << 0, 0, 0, 0;
  int n = estimations.size();
  if(estimations.size() != ground_truth.size() || estimations.size() == 0){
    return RMSE;
  }
  
  for(int i=0; i<n; i++){
    VectorXd residue = estimations[i] - ground_truth[i];
    residue = residue.array() * residue.array();
    RMSE += residue; 
  }
  RMSE = RMSE / n;
  RMSE = RMSE.array().sqrt();
  return RMSE;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
  MatrixXd Hj(3, 4);
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  float j = px*px + py*py;
  float j1 = sqrt(j);
  float j2 = j * j1;
  
  Hj << px/j1, py/j1, 0, 0,
    -py/j, px/j, 0, 0,
    py*(vx*py - vy * px)/j2, px*(vy*px-vx*py)/j2, px/j1, py/j1;
  return Hj;
}
