#include <iostream>
#include "../include/jacobianEstVector.h"
#include "../include/jacobianEst.h"
#include "../include/geomJac.h"
// #include "../include/symm2vec.h"
// #include "../include/spd2vec.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <memory>
#include <chrono>

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

    // test spd2vec with tensor as input with clock//////////////////////////////////
//     using std::chrono::high_resolution_clock;
//     using std::chrono::duration_cast;
//     using std::chrono::duration;
//     using std::chrono::milliseconds;
//     auto t1 = high_resolution_clock::now(); ////////////////
//     Tensor<double,3> T(3,3,4);
//     Tensor<double,3> T_temp;
//     T.setConstant(1);
//     // std::cout<<"T: "<<std::endl<<T<<std::endl;
//     int d = T.dimension(0);
//     int n = T.dimension(2);
//     std::cout<<"T_dimension 0: "<<d<<std::endl;
//     std::cout<<"T_dimension 2: "<<n<<std::endl;
//     VectorXd vec;
//     VectorXd vec_add;
//     MatrixXd M;
//     MatrixXd M_res;
//     array<DenseIndex, 3> offset; 
//     array<DenseIndex, 3> extent;
//     for (int i = 0; i<n; i++){
//         offset = {0, 0, i};
//         extent = {d, d, 1};
//         T_temp = T.slice(offset, extent);
//         std::cout<<"T: "<<std::endl<<T_temp<<std::endl;
//         M = Map<MatrixXd> (T_temp.data(), d, d);
//         std::cout<<"M: "<<std::endl<<M<<std::endl;
//         vec = M.diagonal();
//         for (int j = 1; j<d; j++){
//             vec_add = sqrt(2) * M.diagonal(j);
//             int to_append = vec_add.size();
//             std::cout<<"vec_add size: "<<to_append<<std::endl;
//             std::cout<<"vec_add: "<<std::endl<<vec_add<<std::endl;
//             vec.conservativeResize(vec.size() + vec_add.size(),1);
//             vec.bottomRows(to_append)= vec_add;
//             std::cout<<"vec_size: "<<vec.size()<<std::endl;
//             std::cout<<"vec: "<<std::endl<<vec<<std::endl;
//         }
//         std::cout<<"M size: "<<M_res.size()<<std::endl;
//         M_res.conservativeResize(vec.rows(), i+1);
//         M_res.col(i) = vec;
//     }
//     auto t2 = high_resolution_clock::now(); /////////////////////////
//     /* Getting number of milliseconds as a double. */
//     duration<double, std::milli> ms_double = t2 - t1;
//     std::cout << ms_double.count() << "ms\n";
//     std::cout<<"Result of tensor symm2vec: "<<std::endl<<M_res<<std::endl;
    return 0;
}