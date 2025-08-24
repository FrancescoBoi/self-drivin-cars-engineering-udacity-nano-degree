#include "PID.h"
#include<iostream>
#include <iomanip>
#include <limits>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Kd_, double Ki_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
   this->Kp = Kp_;
   this->Ki = Ki_;
   this->Kd = Kd_;
   counter = 0;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
   //check overflow
   if (counter==std::numeric_limits<double>::max())
   {
       counter = 0.;
       this->i_error = 0.;
   }
   counter += 1.f;
   this->d_error = cte - this->p_error;
   this->p_error = cte;
   this->i_error += cte;
   //std::cout<<"cte: "<<this->p_error<<" -> "<<this->Kp*cte<<std::endl;
   //std::cout<<"cte diff: "<<this->d_error<<" -> "<<this->Kd*this->d_error<<std::endl;
   //td::cout<<"cte intg: "<<this->i_error<<" -> "<<this->Ki*this->i_error<<std::endl;

}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
   //std::cout<<"P: "<<this->Kp*this->p_error<<std::endl;
   //std::cout<<"D: "<<this->Kd*this->d_error<<std::endl;
   return -this->Kp*this->p_error -this->Kd*this->d_error -this->Ki*this->i_error/counter;
}
