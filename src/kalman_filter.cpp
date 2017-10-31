#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;

// for debugging
#include <iostream>
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
  P_ = F_ * P_ * F_.transpose() + Q_;

}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  
  VectorXd y;
  MatrixXd S_;
  MatrixXd K_;
  MatrixXd I_;
  I_ = MatrixXd::Identity(4, 4);

  y = z - H_ * x_;
  S_ = H_ * P_ * H_.transpose() + R_;
  K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * y;
  P_ = (I_ - K_ * H_) * P_;
  

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */

  VectorXd y;
  MatrixXd S_;
  MatrixXd K_;
  MatrixXd I_;
  I_ = MatrixXd::Identity(4, 4);


  // read in position and velocity values
  float px = x_(0);
  float py = x_(1); 
  float vx = x_(2); 
  float vy = x_(3);

  // Calculate position value rho
  float rho = sqrt(px*px+py*py);
  // Calculate angle theta
  float theta = atan2(py,px);
  // Calculate velocity rho dot
  float rho_dot = (px*vx+py*vy)/rho;


  // create prediction vector
  VectorXd z_pred = VectorXd(3);
  z_pred << rho, theta, rho_dot;
  // create y Vector
  y = z - z_pred;

  S_ = H_ * P_ * H_.transpose() + R_;
  K_ = P_ * H_.transpose() * S_.inverse();
  x_ = x_ + K_ * y;
  P_ = (I_ - K_ * H_) * P_;

}
