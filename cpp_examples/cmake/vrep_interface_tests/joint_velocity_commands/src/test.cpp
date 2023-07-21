#include <iostream>
#include "../include/jacobianEstVector.h"
#include "../include/jacobianEst.h"
#include "../include/geomJac.h"
// #include "../include/symm2vec.h"
// #include "../include/spd2vec.h"
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
    // int n = 7;
    VectorXd q_ (7);
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
  
    // test sym2vec///////////////////////////////////////////////
    // Index n_1 = M_test.rows();
    // MatrixXd vec;
    // MatrixXd vec_add;
    // vec = M_test.diagonal();
    // std::cout<<"vec: "<<vec<<std::endl;
    
    // for (Index i =1; i<n_1; i++){
    //     vec_add = sqrt(2)*M_test.diagonal(i);
    //     int to_append = vec_add.size();
    //     std::cout<<"vec_add size: "<<to_append<<std::endl;
    //     vec.conservativeResize(vec.size() + vec_add.size(),1);
    //     vec.bottomRows(to_append)= vec_add;
        
    //     // std::cout<<"vec_before append: "<<vec<<std::endl;
    //     std::cout<<"vec_size: "<<vec.size()<<std::endl;
    //     // vec.bottomRows(to_append) = vec_add;
    //     // vec << vec, vec_add;
    // }
    // test spd2vec with tensor as input
    Tensor<double,3> T(3,3,4);
    T.setConstant(1);
    std::cout<<"T: "<<std::endl<<T<<std::endl;
    int d = T.dimension(0);
    int n = T.dimension(2);
    std::cout<<"T_dimension 0: "<<d<<std::endl;
    std::cout<<"T_dimension 2: "<<n<<std::endl;
    VectorXd vec;
    VectorXd vec_add;
    array<DenseIndex, 3> offset; 
    array<DenseIndex, 3> extent;
    std::array<long,2> shape2 = {3,3};
    
    for (int i =0; i<n; i++){
        
        offset = {0, 0, i};
        extent = {3, 4, 1};
        T = T.slice(offset, extent);
        // here!!!
        MatrixXd M = Map<MatrixXd> (T.data(), d, n);
        std::cout<<"M: "<<std::endl<<M<<std::endl;
        vec = M.diagonal();
        for (int j = 0; j<d-1; j++){
            vec_add = sqrt(2) * vec.diagonal(-1);
            VectorXd vec_jointed(vec.size() + vec_add.size());
            vec << vec ,
                vec_add;
        }
    }

    return 0;
}