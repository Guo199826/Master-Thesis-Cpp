#include "../include/tmprod.h"
// Remember to set Rowmajor tensor!
// // Function to calculate the n-mode product of a tensor and a matrix
Tensor<double, 3> temprod(const Eigen::Tensor<double, 3>& T, const Eigen::MatrixXd& M, const int mode){
    VectorXd size_tens(mode);
    size_tens.setConstant(1);
    int rank = T.rank();
    auto temp = T.size();
    size_tens.head(rank) = T.size();
    return ;
}


    
