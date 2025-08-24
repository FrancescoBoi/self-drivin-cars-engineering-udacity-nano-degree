#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF(){
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
   H_laser_ << 1., 0., 0., 0.,
               0., 1., 0., 0.;



}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
   static const float noise_ax = 9;
   static const float noise_ay = 9;
   long double dt, dt_2 , dt_3, dt_4;

  if (!is_initialized_) {
    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    VectorXd temp_x(4);
    temp_x << 1, 1, 1, 1;
    MatrixXd temp_F(4,4);
    temp_F << 1,0,1,0,
              0,1,0,1,
              0,0,1,0,
              0,0,0,1;

    MatrixXd temp_H;
    //MatrixXd temp_Q(4,4);
    MatrixXd temp_R;
    MatrixXd temp_P(4,4);
    temp_P<<1,0,0,0,
            0,1,0,0,
            0,0,1000,0,
            0,0,0,1000;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // TODO: Convert radar from polar to cartesian coordinates
      //         and initialize state.
      float cs = cos(measurement_pack.raw_measurements_[1]);
      float sn = sin(measurement_pack.raw_measurements_[1]);
      float px = cs* measurement_pack.raw_measurements_[0];
      float py = sn* measurement_pack.raw_measurements_[0];
      float vx = cs*measurement_pack.raw_measurements_[2];
      float vy = sn*measurement_pack.raw_measurements_[2];
      temp_x << px, py, vx, vy;
      cout << "temp_x radar size: "<<temp_x.size()<<endl;
      temp_H = tools.CalculateJacobian(temp_x);
      temp_R = this->R_radar_;
      cout<<"Radar init completed\n";

    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      temp_x<<measurement_pack.raw_measurements_(0), measurement_pack.raw_measurements_(1),
        0, 0;
      temp_H = this->H_laser_;
      temp_R = this->R_laser_;
      cout<<"Lidar init completed\n";

    }
    ekf_.Q_ = MatrixXd(4,4);
    //this->ekf_.Init(temp_x, temp_P, temp_F, temp_H, temp_R, temp_Q);
    this->ekf_.x_ = temp_x;
    this->ekf_.P_ = temp_P;
    this->ekf_.F_ = temp_F;
    this->ekf_.H_ = temp_H;
    this->ekf_.R_ = temp_R;
    this->previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
    return;
  }//end initialisation

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */


   cout << "before predict x_ = " << ekf_.x_ << endl;
   //cout << "before predict  P_ = " << ekf_.P_ << endl;
   dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
   dt_2 = dt * dt;
   dt_3 = dt_2 * dt;
   dt_4 = dt_3 * dt;
   previous_timestamp_ = measurement_pack.timestamp_;
   cout<<"dt = "<<dt<<endl;
   // Modify the F matrix so that the time is integrated
   ekf_.F_(0, 2) = dt;
   ekf_.F_(1, 3) = dt;
   // set the process covariance matrix Q

   ekf_.Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
       0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
       dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
       0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  ekf_.Predict();
  cout << "after predict x_ = " << ekf_.x_ << endl;
  //cout << "after predict  P_ = " << ekf_.P_ << endl;

  /**
   * Update
   */

  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    cout << "before radar update x_ = " << ekf_.x_ << endl;
    cout << "before radar update  P_ = " << ekf_.P_ << endl;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = this->R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    // TODO: Laser updates
    cout << "before laser update x_ = " << ekf_.x_ << endl;
    cout << "before laser update  P_ = " << ekf_.P_ << endl;
    ekf_.H_ = this-> H_laser_;
    ekf_.R_ = this->R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
