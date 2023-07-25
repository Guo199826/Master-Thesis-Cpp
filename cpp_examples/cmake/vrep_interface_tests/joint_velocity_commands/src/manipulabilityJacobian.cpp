#include "../include/manipulabilityJacobian.h"

Tensor<double, 3> manipulabilityJacobian(const MatrixXd &geomJ, const Tensor<double, 3> &J_grad){
    array<int, 3> perm{2,1,3};
    Tensor<double, 3> Jm = tmprod(J_grad, geomJ,2) + tmprod(J_grad.shuffle(perm), geomJ, 1);
    return Jm;
}
