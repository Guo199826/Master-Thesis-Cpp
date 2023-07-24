#include "../include/manipulabilityJacobian.h"

Tensor<double, 3> manipulabilityJacobian(const MatrixXd & geomJ, const Tensor<double, 3> &J_grad){
    Tensor<double, 3> Jm = tmprod(J_grad, geomJ,2) + 
    tmprod(J_grad.shuffle({2,1,3}), geomJ, 1);
}
