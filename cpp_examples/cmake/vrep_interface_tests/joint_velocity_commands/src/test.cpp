#include <iostream>
#include "../include/jacobianEstVector.h"
#include "../include/geomJac.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <memory>

using namespace DQ_robotics;

int main(){
    // robot definition
    auto robot_ptr = std::make_shared<DQ_SerialManipulatorMDH>
            (FrankaEmikaPandaRobot::kinematics());
    DQ_SerialManipulatorMDH robot = DQ_SerialManipulatorMDH(FrankaEmikaPandaRobot::kinematics());

    // Define function handle for geomJac and pose_jacobian
    std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    const VectorXd&, const int)> fct_geomJac_ = geomJac;
    
    // Set link number and joint angle
    int n = 7;
    VectorXd q_ (7);
    q_ << 1.1519, 0.14, 0.2618, 0.0, 0.0, 1.39, 0.0 ; //  validate with q_test in Matlab

    MatrixXd J_sing = jacobianEstVector(geomJac, q_, n, robot);
    std::cout<<"JacobianEst for singular value: "<<std::endl<< J_sing <<std::endl;
    return 0;
}