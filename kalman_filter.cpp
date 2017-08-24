#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

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
  MatrixXd Ft_; 
  Ft_ = F_.transpose();
  cout << P_ << endl;
  P_ = F_ * P_ * Ft_ + Q_ ;
  cout << "Complete predict step" << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  cout << "Entering update function " << endl;
  VectorXd y = z - H_ * x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_ ;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new state
  x_ = x_ + K * y;

  //size of P_ matrix;
  int size = sizeof P_;
  MatrixXd I = MatrixXd::Identity(4,4);

  P_ = (I - K * H_) * P_ ;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  cout << "Starting UpdateEKF" << endl;
  float x_tmp = x_(0);
  float y_tmp = x_(1);
  float vx_tmp = x_(2);
  float vy_tmp = x_(3);

  float rho = sqrt(x_tmp*x_tmp + y_tmp*y_tmp);
 
  float theta;
  
  //avoid division by zero
  theta = 0;
  if (fabs(x_tmp) < 0.0001) {
    cout << "Error whilte coverting to polar coordinates: Division by zero" << endl;  
  } else {
    theta = atan2(y_tmp,x_tmp);
  }
        
  float ro_dot =  ( x_tmp * vx_tmp + y_tmp * vy_tmp)/ rho;

  cout << "rho theta ro_dot" << endl;
  cout << rho << theta << ro_dot << endl;

  VectorXd Zpred = VectorXd(3);
  Zpred << rho, theta, ro_dot; 

  VectorXd y = z - Zpred ;
  y(1) = atan2(sin(y(1)),cos(y(1)));

  while (y(1) < -M_PI) 
         y(1) = y(1) + 2*M_PI;
  while (y(1) > M_PI) 
         y(1) = y(1) - 2*M_PI;
	
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_ ;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_ * Ht * Si;

  // new state
  x_ = x_ + K * y;

  //size of P_ matrix;
  int size = sizeof P_;
  MatrixXd I = MatrixXd::Identity(4,4);

  P_ = (I - K * H_) * P_ ; 
  cout << "Done UpdateEKF" << endl;
}
