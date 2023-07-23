#ifndef MANIPULABILITYJACOBIAN_H
#define MANIPULABILITYJACOBIAN_H
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>

using namespace Eigen;
Tensor<double, 3> temprod(const Eigen::Tensor<double, 3>& T, const Eigen::MatrixXd& M, const int mode);

#endif