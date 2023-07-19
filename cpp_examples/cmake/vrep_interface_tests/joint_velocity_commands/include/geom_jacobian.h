#include <iostream>
#include <eigen3/Eigen/Dense>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

MatrixXd geomJac(const DQ_SerialManipulator &robot, const MatrixXd &poseJacobian, const VectorXd &q, const int n);