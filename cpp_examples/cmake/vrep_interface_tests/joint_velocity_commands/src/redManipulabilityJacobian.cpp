#include "../include/manipulabilityJacobian.h"

MatrixXd redManipulabilityJacobian(const MatrixXd & geomJ, const Tensor<double, 3> &J_grad){
    Tensor<double, 3> Jm = manipulabilityJacobian(geomJ, J_grad);
    MatrixXd Jm_red(21,7);
    Matrix<double,6,6> M_temp;
    int size = Jm.dimension(2); // should be 7
    for(int i=0; i<size; i++){
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {6, 6, 1};
        Jm.slice(offset, extent) = J_result_m2t;
        M_temp = Map<Matrix<double,6,6>> (Jm.slice(offset, extent) = J_result_m2t.data(), 6, 6);
        VectorXd Jm_red_i = spdToVec_vec(M);
        Jm_red.col(i)= Jm_red_i;
    }
    std::cout<<"redManiJacobian: "<<std::endl<<Jm_red<<std::endl;
    return Jm_red;
}