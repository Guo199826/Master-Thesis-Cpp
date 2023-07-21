#include "../include/spd2vec.h"

// using Mandel notation from Noemie's code
VectorXd spd2vec (const MatrixXd &mat){
    int n = mat.rows();
    VectorXd vec;
    VectorXd vec_add;
    vec = mat.diagonal();
    for (int i =0; i<n; i++){
        vec_add = sqrt(2) * mat.diagonal()
        VectorXd vec_jointed(vec.size() + )
        vec = v
    }

    VectorXd res(mat.rows()*(mat.cols()+1)/2);
    Index size = mat.rows();
    Index offset = 0;
    for(Index j=0; j<mat.cols(); ++j) {
        res.segment(offset,size) = mat.col(j).tail(size);
        offset += size;
        size--;
    }   
    return res;
}
