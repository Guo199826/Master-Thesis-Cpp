#include <iostream>
#include "../include/jacobianEstVector.h"
#include "../include/jacobianEst.h"
#include "../include/geomJac.h"
// #include "../include/tmprod.h"
// #include "../include/symm2vec.h"
// #include "../include/spd2vec.h"
#include <dqrobotics/DQ.h>
#include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include <memory>
#include <array>

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
    
    // Tensor<double,3> T(3,3,4);
    // T=T.random();
    // std::cout<<"Tensor original: "<<std::endl<<T<<std::endl;
    // // PermutationBase<Tensor<double,3>> perm;
    // // perm.indices() = {2,1,0};
    // // Tensor<double,3> T_temp = perm*T(:,:,1);
    // // std::array<Index, 3> indices {4, 3, 3};
    // // T.resize(indices,);
    // array<int, 3> shuffling({1,2,0});
    // Tensor<double,3> T_temp = T.shuffle(shuffling);
    // std::cout<<"Tensor permuted : "<<std::endl<<T_temp<<std::endl;
    // auto T_ = T.reshape(indices,);
    // std::cout<<"Tensor resized: "<<std::endl<<T_<<std::endl;

    // test tmprod/////////////////////////////////////////////////////////////////////
    // Eigen::Tensor<double, 3> tensor(2, 3, 4);
    // tensor.setRandom();
    
    // Eigen::MatrixXd matrix(3, 2);
    // matrix.setRandom();
    
    // int mode = 1; // Choose the mode for the n-mode product (0, 1, or 2)
    
    // Eigen::Tensor<double, 3> result = tmprod(tensor, matrix, mode);


    // Example usage
    Tensor<double, 3> T(3, 3, 3);
    Tensor<double, 3> result;
    Tensor<double, 3> T_temp;
    T.setConstant(1.0);
    int mode =2;
    MatrixXd matrix(3, 3);
    matrix << 1, 2, 3,
              4, 5, 6,
              7, 8, 9;

    array<long, 3> size_tens;
    int rank = T.rank(); //=3
    for (int i = 0; i<rank; i++){
        int temp = T.dimension(i);
        size_tens[i] = temp;
    }
    array<long, 3> perm;
    switch (mode) {
    case 1:
        perm = {0,1,2};
        break;

    case 2:
        perm = {1,0,2};
        break;

    case 3:
        perm = {2,0,1};
        break;
    }
    size_tens={size_tens[perm[0]], size_tens[perm[1]], size_tens[perm[2]]};
    T_temp = T;
    if (mode != 1){
        result =T_temp.shuffle(perm);
        
    }
    std::cout<<"result: "<<result<<std::endl;
    // nmode product:
    size_tens[0] = matrix.rows();
    int col = result.size()/result.dimension(0);
    MatrixXd M_temp = Map<MatrixXd> (result.data(), result.dimension(0),col);
    MatrixXd M_temp2 = matrix* M_temp;
    std::cout<<"M_temp2: "<<M_temp2<<std::endl;
    auto T_temp2 = TensorMap<Tensor<double,3,RowMajor>>(M_temp2.data(), size_tens[0],size_tens[1],size_tens[2]);
    // // // result = T_temp.reshape(size_tens);
    std::cout << "Result of n-mode product:" << std::endl<<T_temp2<< std::endl;
    std::cout << "Result of n-mode product at:" << std::endl<<T_temp2(2,1,3)<< std::endl;


    return 0;
}