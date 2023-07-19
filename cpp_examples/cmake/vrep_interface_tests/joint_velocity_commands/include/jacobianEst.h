#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <functional>

using namespace Eigen;
using namespace DQ_robotics;

MatrixXd jacobianEst(std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    const VectorXd&, const int)> fct_geomJac, const VectorXd& q, 
    std::function<MatrixXd(const VectorXd&)> fct_J);