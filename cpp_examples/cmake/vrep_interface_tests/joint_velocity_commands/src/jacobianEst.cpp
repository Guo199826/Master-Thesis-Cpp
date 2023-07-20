#include "../include/jacobianEst.h"

Tensor<double> jacobianEst(std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    const VectorXd&, const int)> fct_geomJac, const VectorXd& q, 
    std::function<MatrixXd(const VectorXd&)> fct_J,
    const int n) 
{
    double q_delta = 0.001;
    VectorXd q_add(n);
    q_add(0) = q_delta;
    VectorXd q_ii = q + q_add;
    VectorXd q_i = q - q_add;

    for (int i=0; i<n; i++){

    }
    
}
    
