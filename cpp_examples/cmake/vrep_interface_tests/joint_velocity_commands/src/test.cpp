#include <iostream>
#include "../include/jacobianEstVector.h"
#include "../include/jacobianEst.h"
#include "../include/geomJac.h"
// #include "../include/symm2vec.h"
#include "../include/spd2vec.h"
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
    VectorXd q_ (n);
    q_ << 1.1519, 0.14, 0.2618, 0.0, 0.0, 1.39, 0.0 ; //  validate with q_test in Matlab

    // MatrixXd J_sing = jacobianEstVector(geomJac, q_, n, robot);
    // std::cout<<"JacobianEst for singular value: "<<std::endl<< J_sing <<std::endl;

    // Tensor<double, 3> J_jacobian = jacobianEst(geomJac, q_, n, robot);

    //test symm2vec
    MatrixXd M_test(3,3);
    M_test<< 3, 4, 7,
             4, 1, 2,
             7, 2, 5;

    // VectorXd vec_test = symm2vec(M_test);
    // std::cout<<"vec_test: "<<std::endl<<vec_test<<std::endl;

    return 0;
}