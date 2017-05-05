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
  MatrixXd Hj(3, 4);
  // state paremeters
  float px = x_state(0);
  float py = x_state(1);
  float vx = x_state(2);
  float vy = x_state(3);

  //check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}
