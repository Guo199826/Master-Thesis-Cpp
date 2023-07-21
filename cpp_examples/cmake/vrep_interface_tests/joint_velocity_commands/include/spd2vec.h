#ifndef SPD2VEC_H
#define SPD2VEC_H

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>

using namespace Eigen;

VectorXd spd2vec (const MatrixXd &mat);

#endif