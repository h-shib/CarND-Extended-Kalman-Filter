#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  // validate inputs
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

  if ((estimations.size() == 0) ||
  		(estimations.size() != ground_truth.size())) {
  	cout << "Invalid inputs for calculating RMSE." << endl;
  	return rmse;
  }

  // calculate the RMSE
  for (unsigned int i=0; i<estimations.size(); ++i) {
  	VectorXd residual = estimations[i] - ground_truth[i];
  	residual = residual.array()*residual.array();
  	residual += residual;
  }
  rmse = rmse / estimations.size();
  rmse = sqrt(rmse);
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    * Calculate a Jacobian here.
  */
}
