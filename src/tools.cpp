#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth)
{
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  //
  // the estimation vector should be the same size as the ground_truth vector
  // if not, return intialized rmse.
  //
  if ( (estimations.size() == 0) || (estimations.size() != ground_truth.size()))
  {
    std::cout << "Warning: Size error: estimations and ground_truth vectors are not the same size. \n";
    return rmse;
  }

  //
  // Accumaulate the difference between the estimated
  // and ground truth values.
  //
  for(int i=0; i < estimations.size(); ++i)
  {
    VectorXd est = estimations[i];
    VectorXd truth = ground_truth[i];

    VectorXd residual = est - truth;
    VectorXd squared = residual.array() * residual.array();

    rmse += squared;
  }

  //
  // calculate the mean
  //
  rmse /= estimations.size();

  //
  // calculate the squared root and return
  //
  //return rmse.array().sqrt();
  rmse = rmse.cwiseSqrt();

  std::cout << "{" << rmse(0) << ", " << rmse(1) << ", " << rmse(2) << ", "  << rmse(3) << "}\n";
  return rmse;

}

//
// computes and returns the Jacobian matrix
//
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state)
{
  MatrixXd Hj = MatrixXd::Zero(3,4);

  double px = x_state(0);
  double py = x_state(1);
  double vx = x_state(2);
  double vy = x_state(3);

  double c1 = px * px + py * py;
  double c2 = sqrt(c1);
  double c3 = c1 * c2;
  double vxpy = vx * py;
  double vypx = vy * px;

  //
  //check division by zero
  //
  if (fabs(c1 < 0.0001))
  {
    std::cout << "CalculateJacobian - Division by Zero\n";
  }
  else
  {
    Hj << px / c2,   py / c2,  0,   0,
          -py / c1,    px / c1,    0,   0,
          py * (vxpy - vypx) / c3,  px * (vypx - vxpy) / c3,  px / c2,  py / c2;
  }

  return Hj;
}
