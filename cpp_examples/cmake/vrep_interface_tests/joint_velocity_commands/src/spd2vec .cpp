#include "../include/spd2vec.h"

// using Mandel notation from Noemie's code
VectorXd spd2vec_vec (const MatrixXd &M){
    Index n_1 = M.rows();
    VectorXd vec;
    VectorXd vec_add;
    vec = M.diagonal();
    std::cout<<"vec: "<<vec<<std::endl;
    
    for (Index i =1; i<n_1; i++){
        vec_add = sqrt(2)*M.diagonal(i);
        int to_append = vec_add.size();
        std::cout<<"vec_add size: "<<to_append<<std::endl;
        vec.conservativeResize(vec.size() + vec_add.size());
        vec.bottomRows(to_append)= vec_add;
        
        // std::cout<<"vec_before append: "<<vec<<std::endl;
        std::cout<<"vec_size: "<<vec.size()<<std::endl;
        // vec.bottomRows(to_append) = vec_add;
        // vec << vec, vec_add;
    }
    // std::cout<<"spd2vec_vec_test: "<<std::endl<<vec<<std::endl;
    return vec;
}

MatrixXd spd2vec (const Tensor<double,3> &T){
    int d = T.dimension(0);
    int n = T.dimension(2);
    VectorXd vec;
    VectorXd vec_add;
    array<DenseIndex, 3> offset; 
    array<DenseIndex, 3> extent;
    
    for (int i =0; i<n; i++){
        
        offset = {0, 0, i};
        extent; = {6, 7, 1};

        vec = T.slice(offset, extent).diago
        for (int j = 0; j<d-1; j++){
            vec_add = sqrt(2) * T.diagonal(-1);
            VectorXd vec_jointed(vec.size() + vec_add.size());
            vec << vec ,
                vec_add;
        }
        
    }
    return vec;
}