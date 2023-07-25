#include <iostream>
#include "../include/jacobianEstVector.h"
#include "../include/jacobianEst.h"
#include "../include/geomJac.h"
#include "../include/manipulabilityJacobian.h"
// #include "../include/tmprod.h"
// #include "../include/symm2vec.h"
// #include "../include/spd2vec.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
// #include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>
// #include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
#include <memory>
#include <array>
// #include "osqp/osqp.h"
#include "OsqpEigen/OsqpEigen.h"

using namespace DQ_robotics;
// int test();

int main(){
    // // robot definition
    // auto robot_ptr = std::make_shared<DQ_SerialManipulatorMDH>
    //         (FrankaEmikaPandaRobot::kinematics());
    // DQ_SerialManipulatorMDH robot = DQ_SerialManipulatorMDH(FrankaEmikaPandaRobot::kinematics());

    // // Define function handle for geomJac and pose_jacobian
    // std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    // const VectorXd&, const int)> fct_geomJac_ = geomJac;
    
    // // test J and geomJ
    // // Set link number and joint angle
    // int n = 7;
    // VectorXd q_ (7);
    // q_ << 1.1519, 0.14, 0.2618, 0.0, 0.0, 1.39, 0.0 ; //  validate with q_test in Matlab
    // MatrixXd J = robot_ptr->pose_jacobian(q_);
    // std::cout<<"J: "<<std::endl<<J<<std::endl;
    // MatrixXd J_geom = geomJac(robot, J, q_, n);
    // std::cout<<"J_geom: "<<std::endl<<J_geom<<std::endl;

    // test jacobianEstVector
    // ev_diff: eigenvalue of Manipulability
    // MatrixXd J_sing = jacobianEstVector(geomJac, q_, n, robot); 
    // std::cout<<"JacobianEst for singular value: "<<std::endl<< J_sing <<std::endl;
    
    // test jacobianEst
    // Tensor<double, 3> J_jacobian = jacobianEst(geomJac, q_, n, robot);

    //test symm2vec
    // MatrixXd M_test(3,3);
    // M_test<< 3, 4, 7,
    //          4, 1, 2,
    //          7, 2, 5;
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
    
    // test tmprod/////////////////////////////////////////////////////////////////////
    // Tensor<double, 3> T(3,3,3);
    // T.setConstant(1.0);
    // MatrixXd matrix(3, 3);
    // matrix << 1, 2, 3,
    //           4, 5, 6,
    //           7, 8, 9;
    // int mode = 1; // Choose the mode for the n-mode product (0, 1, or 2)
    // Tensor<double, 3> result = tmprod(T, matrix, 2);
    // std::cout << "Result of n-mode product:" << std::endl<<T_temp2<< std::endl;
    // std::cout << "Result of n-mode product test; :" << std::endl<<T_temp2(2,1,3)<< std::endl;

    // test logmap
    // MatrixXd M_1(6,6);
    // M_1 <<          2.1628,   -0.1748,    0.0359,   -0.2370,   -0.7552,   -0.0315,
    //                 -0.1748,    2.8573,    0.3590,    0.8642,    0.1461,    0.0609,
    //                 0.0359,    0.3590,    1.9799,   -0.0817,   -0.2148,    0.0909,
    //                 -0.2370,    0.8642,   -0.0817,    0.4238,    0.1317,    0.0448,
    //                 -0.7552,    0.1461,   -0.2148,    0.1317,    0.4455,   -0.0576,
    //                 -0.0315,    0.0609,    0.0909,    0.0448,   -0.0576,    0.0644;
    // MatrixXd M_2(6,6);
    // M_2 <<          2.7930,    0.1532,   -0.1684,    0.1195,   -0.0701,    0.7743,
    //                 0.1532,    2.3853,   -0.1840,    0.1339,   -0.0114,   -0.2043,
    //                 -0.1684,   -0.1840,    1.8217,   -0.2919,   -0.0225,   -0.1081,
    //                 0.1195,    0.1339,   -0.2919,    0.2293,   -0.0415,    0.0601,
    //                 -0.0701,   -0.0114,   -0.0225,   -0.0415,    0.0164,   -0.0309,
    //                 0.7743,   -0.2043,   -0.1081,    0.0601,   -0.0309,    0.3876;
    
    // test redManiJac
    //tbc


    /* // test OSQP solver
    constexpr double tolerance = 1e-4;
    Eigen::Matrix<c_float, 2, 2> H;
    Eigen::SparseMatrix<c_float> H_s;
    H << 4, 1,
         1, 2;
    H_s = H.sparseView();
    H_s.pruned(0.01); // set those who smaller than 0.01 as zero?

    // Eigen::SparseMatrix<c_float> H_s(2,2);
    // H_s.insert(0,0) = 4;
    // H_s.insert(0,1) = 1;
    // H_s.insert(1,0) = 1;
    // H_s.insert(1,1) = 2;

    Eigen::SparseMatrix<c_float> A_s(3,2);
    A_s.insert(0,0) = 1;
    A_s.insert(0,1) = 1;
    A_s.insert(1,0) = 1;
    A_s.insert(2,1) = 1;
    std::cout<<"H_S : "<<std::endl<<H_s<<std::endl;
    Eigen::Matrix<c_float, 2, 1> gradient;
    gradient << 1, 1;
    Eigen::Matrix<c_float, 3, 1> lowerBound;
    lowerBound << 1, 0, 0;
    Eigen::Matrix<c_float, 3, 1> upperBound;
    upperBound << 1, 0.7, 0.7;

    OsqpEigen::Solver solver;
    //settings:
    // solver.settings()->setVerbosity(true); // print outptu or not
    solver.settings()->setAlpha(1.0); // ADMM relaxation parameter/step size/penalty parameter
    
    solver.data()->setNumberOfVariables(2);
    solver.data()->setNumberOfConstraints(3);
    solver.data()->setHessianMatrix(H_s);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(A_s);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);

    solver.initSolver();
    solver.solveProblem();
    // bool flag = solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
    // std::cout<<"No error: "<<flag<<std::endl;
    Eigen::Matrix<c_float, 2, 1> expectedSolution;
    expectedSolution << 0.3,  0.7;

    VectorXd solution = solver.getSolution();
    std::cout<<"Solution : "<<std::endl<<solution<<std::endl;

    bool converge = solver.getSolution().isApprox(expectedSolution, tolerance);
    std::cout<<"Converged to desired solution: "<<converge<<std::endl;
    */ 

    return 0;
}