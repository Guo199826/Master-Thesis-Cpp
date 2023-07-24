// Not the same as in Matlab
// input arguments are Matrix instead of Tensor
#include "../include/logmap.h"

MatrixXd logmap(const MatrixXd &M_1, const MatrixXd &M_2){
    int size = M_1.rows();
    EigenSolver<MatrixXd> eigensolver(M_1/M_2);
    MatrixXd EV = eigensolver.eigenvalues().asDiagonal();
    MatrixXd EVec = eigensolver.eigenvectors();
}