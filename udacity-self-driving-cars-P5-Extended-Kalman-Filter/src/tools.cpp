#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
    const vector<VectorXd> &ground_truth) {

    VectorXd rmse(4);
    rmse << 0,0,0,0;

    // TODO: YOUR CODE HERE
    // check the validity of the following inputs:
    //  * the estimation vector size should not be zero
    //  * the estimation vector size should equal ground truth vector size
    if (estimations.size()==0)
    {
        std::cout<<"Estimation vector is 0\n";
        return rmse;
    }
    if (estimations.size()!=ground_truth.size())
    {
        std::cout<<"Ground truth has different dimensions from estimations\n";
        return rmse;
    }
    // TODO: accumulate squared residuals
    else
    {
        for (unsigned int i=0; i < estimations.size(); ++i)
        {
          VectorXd temp = estimations[i] - ground_truth[i];
          temp = temp.array()*temp.array();
          rmse += temp;
        }
        rmse /= estimations.size();
        rmse = rmse.array().sqrt();
    }
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
    MatrixXd Hj(3,4);
    // recover state parameters
    float px = x_state(0);
    float py = x_state(1);
    float vx = x_state(2);
    float vy = x_state(3);

    // TODO: YOUR CODE HERE

    // check division by zero
    if (abs(px)<0.0000001 && abs(py)<0.0000001)
    {
        std::cout<<"Division by 0\n";
        Hj<<0,0,0,0,0,0,0,0,0,0,0,0;
    }
    else
    {
    // compute the Jacobian matrix
        float sqrd_px_py = pow(px,2) + pow(py,2);
        float sqrt_px_py = pow(sqrd_px_py, 0.5);
        Hj<< px/sqrt_px_py, py/sqrt_px_py, 0, 0,
            -py/sqrd_px_py, px/sqrd_px_py, 0, 0,
            py*(vx*py-vy*px)/pow(sqrd_px_py, 1.5), px*(vy*px-vx*py)/pow(sqrd_px_py, 1.5),
                px/sqrt_px_py, py/sqrt_px_py;
    }
    return Hj;
}
