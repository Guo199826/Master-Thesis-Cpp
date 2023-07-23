#include "../include/tmprod.h"

Tensor<double, 3> tmprod(const Tensor<double, 3>&T, const MatrixXd &M, const int mode){
    Tensor<double, 3> S = T;
    if (mode != 1) {
        S = PermutationBase<Tensor<double, 3>>(S)
    }
}