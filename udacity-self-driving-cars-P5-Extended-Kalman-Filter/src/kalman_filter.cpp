#include "kalman_filter.h"
#include <math.h>
#include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

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
   * TODO: predict the state
   */
   this->x_ = this->F_* this->x_;
   //P = F P F T + Q
   this->P_ = this->F_*this->P_*this->F_.transpose()+this->Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
   static const MatrixXd I = MatrixXd::Identity(4,4);
   static const MatrixXd Ht = this->H_.transpose();
   VectorXd y = z - this->H_* this->x_;
   MatrixXd S_ = this->H_ * this->P_ * Ht + this->R_;
   MatrixXd K_ = this->P_ * Ht * S_.inverse();
   this->x_ += K_*y;
   this->P_ = (I-K_ * this->H_)*this->P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
   static const MatrixXd I = MatrixXd::Identity(4,4);

   float rho = sqrt(pow(this->x_[0],2)+pow(this->x_[1],2));
   float phi = atan2(this->x_[1], this->x_[0]);
   float rho_dot = (this->x_[0]*this->x_[2] + this->x_[1]*this->x_[3])/rho;
   VectorXd polar_coords(3);
   polar_coords<<rho, phi, rho_dot;
   VectorXd y = z - polar_coords;
   //float angle_ = y[1];
   while (y[1]<-M_PI)
     y[1] += 2*M_PI;
   while (y[1]>M_PI)
     y[1]-= 2*M_PI;
   //y[1) = angle_;
   MatrixXd Ht = this->H_.transpose();

   MatrixXd S_ = this->H_ * this->P_ * Ht + this->R_;
   MatrixXd K_ = this->P_ * Ht * S_.inverse();
   this->x_ += K_*y;
   this->P_ = (I-K_ * this->H_)*this->P_;

}
