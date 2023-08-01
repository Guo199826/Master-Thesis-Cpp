// File: Admittance controller according to the external force
// Contributor: Yuhe Gong
// Email: gongyuhe1023@gmail.com
#include "../include/admittance_controller.h"


// Function: Inertia Matrix
    // Input:
    // Output: 
        // M_I: inertia matrix of Franke Emika
//MatrixXd Inertia_Matrix(
//    )
//{
//    // M_I: inertia matrix, accessing from Franka Emika
//    // TODO: need to change the joint space intertia matrix to the task space one. (through J)
//    Matrix<double, 6, 6> M_I;
//    M_I.setIdentity();
//    return  M_I;
//}


// Function: External Force
    // Input:
    // Output: 
        // F_ext: extrnal force
//VectorXd External_Force(
//    )
//{
//    // F_ext: external force, sensoring from Franke Emika
//    // TODO: Should change to external force sensoring from Franke Emika
//    VectorXd F_ext;
//    F_ext.setOnes(6);
//    return F_ext;
//}


// Function: Admittance Controller
    // Input: 
        // x_d: desired end-effector position
        // x_c: current end-effector position
        // x_dot_c: current end-effector velocity
    // Output: 
        // x_d_modified: modified desired end-effector position
VectorXd Admittance_Controller(
    const VectorXd& x_d, 
    const VectorXd& x_c, 
    const VectorXd& x_dot_c
    )
{

    // Controller Variable:
    // D_d: desired damping factor
    Matrix<double, 6, 6> D_d;
    D_d.setIdentity();
    // K_d: desired stiffnees factor
    Matrix<double, 6, 6> K_d;
    K_d.setIdentity();
    
    // Get inertia matrix and external force
    Matrix<double, 6, 6> M_I;
    M_I.setIdentity();
    VectorXd F_ext;
    F_ext.setOnes(6);

    // Controll Signal
    VectorXd x_ddot;
    x_ddot = M_I.inverse() * (F_ext - K_d * (x_c - x_d) - D_d * x_dot_c);
    std::cout << "M_I.inverse(): " <<std::endl << M_I.inverse() << std::endl;
    std::cout << "F_ext: " <<std::endl << F_ext << std::endl;
    std::cout << "K_d: " <<std::endl << K_d << std::endl;
    std::cout << "x_c - x_d: " <<std::endl << x_c - x_d << std::endl;
    std::cout << "D_d: " <<std::endl << D_d << std::endl;
    std::cout << "x_dot_c: " <<std::endl << x_dot_c << std::endl;

    // Modified desired position
    // Assum t = 1
    float t;
    t = 1;
    VectorXd x_modified_d;
    x_modified_d = x_d - x_ddot * t * t;

    std::cout << "dampling factor: " <<std::endl << D_d << std::endl;

    return x_modified_d;

}