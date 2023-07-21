#ifndef SPD2VEC_H
#define SPD2VEC_H

#include <iostream>
#include <eigen3/Eigen/Dense>

using namespace Eigen;

VectorXd spd2vec (const MatrixXd &mat);

#endif