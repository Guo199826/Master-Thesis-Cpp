#ifndef ADMITCONTROLLER_H
#define ADMITCONTROLLER_H
#include <iostream>
#include <fstream>
#include <eigen3/Eigen/Dense>
#include <eigen3/unsupported/Eigen/CXX11/Tensor>
#include <dqrobotics/DQ.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>


#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include "../include/FrankaRobot.h"
// #include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
// #include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "../include/dq2tfm.h"
#include "../include/logmap.h"
#include "../include/jacobianEst.h"
#include "../include/jacobianEstVector.h"
#include "../include/geomJac.h"
#include "../include/manipulabilityJacobian.h"


using namespace Eigen;
using namespace DQ_robotics;

VectorXd Admittance_Controller(
    const VectorXd& x_d, 
    const VectorXd& x_c, 
    const VectorXd& x_dot_c
    );

//VectorXd Inertia_Matrix(
//    );

//VectorXd External_Force(
//    );

#endif