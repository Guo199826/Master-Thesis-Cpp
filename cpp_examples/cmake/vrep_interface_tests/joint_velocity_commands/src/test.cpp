#include <iostream>
#include "../include/jacobianEstVector.h"
#include "../include/geomJac.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <memory>

using namespace DQ_robotics;

MatrixXd DQ_SerialManipulator::pose_jacobian(const VectorXd &q_vec);

int main(){
    // robot definition
    auto robot = std::make_shared<DQ_SerialManipulatorMDH>
            (FrankaEmikaPandaRobot::kinematics());

    // Define function handle for geomJac and pose_jacobian
    std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    const VectorXd&, const int)> fct_geomJac_ = geomJac;
    std::function<MatrixXd(const VectorXd&)> fct_J_ = robot->pose_jacobian;
    // Set joint angle
    VectorXd q (7);
    q << 0.0,0.0,0.0,0.0,0.0,0.0,0.0 ;
    // DQ Jacobian
        MatrixXd J = robot->pose_jacobian(q);

    return 0;
}