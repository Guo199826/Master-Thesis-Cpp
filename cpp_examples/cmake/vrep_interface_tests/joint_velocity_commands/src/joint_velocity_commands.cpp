/**
(C) Copyright 2022 DQ Robotics Developers
This file is part of DQ Robotics.
    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.
    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.
    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.
Contributors:
- Juan Jose Quiroz Omana (juanjqo@g.ecc.u-tokyo.ac.jp)
- Murilo Marques Marinho (murilo@g.ecc.u-tokyo.ac.jp)

Instructions:
Prerequisites:
- dqrobotics
- dqrobotics-interface-vrep

1) Open the CoppeliaSim scene joint_velocity_commands.ttt
2) Be sure that the Lua script attached to the object DQRoboticsApiCommandServer is updated.
   (Updated version: vrep_interface_tests/DQRoboticsApiCommandServer.lua)
3) Compile, run and enjoy!
*/
#include <iostream>
// #include <dqrobotics/DQ.h>
#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
#include "../include/FrankaRobot.h"
// #include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
// #include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "../include/dq2tfm.h"
#include "../include/logmap.h"
#include "../include/jacobianEst.h"
#include "../include/jacobianEstVector.h"
#include "../include/geomJac.h"
#include "../include/manipulabilityJacobian.h"
#include "../include/franka_analytical_ik-main/franka_ik_He.hpp"
// #include "osqp/osqp.h"
#include "OsqpEigen/OsqpEigen.h"

using namespace Eigen;
using namespace DQ_robotics;

int main(void)
{
    // Initialize V-REP interface
    DQ_VrepInterface vi;
    vi.connect(19997,100,10);
    vi.set_synchronous(true);
    std::cout << "Starting V-REP simulation..." << std::endl;
    vi.start_simulation();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::vector<std::string>jointnames = {"Franka_joint1", "Franka_joint2",
                                           "Franka_joint3", "Franka_joint4",
                                           "Franka_joint5", "Franka_joint6",
                                           "Franka_joint7"};

    // Robot definition
    DQ_SerialManipulatorMDH robot = FrankaRobot::kinematics();
    std::shared_ptr<DQ_SerialManipulatorMDH> robot_ptr = std::make_shared<DQ_SerialManipulatorMDH> (robot);
    DQ_SerialManipulatorMDH robot_ik = FrankaRobot::kinematics();
    DQ offset_base = 1 + E_ * 0.5 * DQ(0, 0, 0, 0);
    robot_ik.set_base_frame(offset_base);
    robot_ik.set_reference_frame(offset_base);
    
    std::cout<<"IK: Originally base frame in Vrep: ..."<< robot_ik.get_base_frame()<<std::endl;
    std::cout<<"IK: Now base frame in Vrep:.... "<< robot_ik.get_effector()<<std::endl;

    DQ base_frame = vi.get_object_pose("Franka_joint1");
    DQ eef_frame = vi.get_object_pose("Franka_connection");
    std::cout<<"Originally base frame in Vrep:.... "<<robot.get_base_frame()<<std::endl;
    std::cout<<"base frame in Vrep: "<<base_frame<<std::endl;
    std::cout<<"eef frame in Vrep: "<<eef_frame<<std::endl;
    robot.set_base_frame(base_frame);
    robot.set_reference_frame(base_frame);
    std::cout<<"Now base frame in Vrep:.... "<<robot.get_base_frame()<<std::endl;
    std::cout<<"Now eef frame in Vrep:.... "<<robot.get_effector()<<std::endl;

    // Set link number and joint angle
    int n = 7;
    VectorXd q_ (n);
    VectorXd q_1 (n);
    q_1 << 1.15192, 0.383972, 0.261799, -1.5708, 0.0, 1.39626, 0.0 ; // validate with q_test in Matlab
    q_ << -1.98968, -0.383972, -2.87979, -1.5708, 4.20539e-17, 1.39626, 0;
    // q_ << -2.20775, -0.443532, -2.67585, -1.51926, -0.102954, 1.36896, 0;
    // q_ << 1.42395, 0.349786, 0.0102086, -1.60643, 0.101047, 1.41151, 0 ;
    // one pair:
    // q_1 << 0, 0.384, 0, -1.5708,  0, 1.3963, 0.0 ;
    // q_<< -3.37246e-16, 0.534369, 4.37472e-16, -1.33784, -5.36556e-16, 1.31371, 0;
    // q_ << 0, 0, 0, -1.5708, 0.0, 1.3963, 0.0 ;
    // DQ x_val = robot.fkm(q_);
    // std::cout<<"----------Fkm original-----------"<<std::endl<<x_val<<std::endl;
    // DQ x_0 = robot.fkm(q_1);
    // std::cout<<"----------Fkm of the q from IK----"<<std::endl<<x_0<<std::endl;
    int m = 6; // Dimension of workspace

    // // Auxiliar variables
    double dt = 1E-2;	// Time step
    int nbIter = 3; // Number of iterations (orig: 65)
    int nbData = 1; // no trajectory
    int t_all = nbIter*nbData;

    // Desired cartesian and manipulability pos/trajectory
    VectorXd q_goal(n);
    q_goal << -pi/2.0, 0.004, 0.0, -1.57156, 0.0, 1.57075, 0.0;
    MatrixXd J_goal;
    MatrixXd J_geom_goal;
    MatrixXd Me_d;
    J_goal = robot.pose_jacobian(q_goal);
    J_geom_goal = geomJac(robot, J_goal, q_goal, n);
    Me_d = J_geom_goal*J_geom_goal.transpose();

    // Define function handle for geomJac and pose_jacobian
    std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    const VectorXd&, const int)> fct_geomJac_ = geomJac;
    
    // change joint angle commands_
    // VectorXd q = vi.get_joint_positions(jointnames);
    VectorXd q_0 = vi.get_joint_positions(jointnames);
    std::cout << "Joint positions q (at starting) is: \n"<< std::endl << q_0 << std::endl;
    vi.set_joint_positions(jointnames,q_);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // vi.set_joint_positions(jointnames,q_1);
    // std::cout<<"change joint angle to orig..."<<std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    VectorXd q = vi.get_joint_positions(jointnames);
    std::cout << "Now Joint positions q is: \n"<< std::endl << q << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    // test ik solver (without offset to synchronize with Vrep)
    // DQ x = robot_ik.fkm(q).normalize();
    // Matrix4d M_tf = dq2tfm(x);
    // std::array<double, 16> arr_tf;
    // std::copy(M_tf.data(), M_tf.data() + 16, arr_tf.begin());
    // double q7 = 0.0;
    // std::array<double, 7> qt_arr;
    // std::copy(q.data(), q.data() + q.size(), qt_arr.begin());
    // std::cout << "current q" << std::endl;
    // for (double val : qt_arr) {
    //     std::cout << val << " ";
    // }
    // std::cout << std::endl;
    // std::array<std::array<double, 7>,4> q_est = franka_IK_EE (arr_tf,
    //                                                 q7, qt_arr );
    // std::array<double, 7> q_est = franka_IK_EE_CC (arr_tf,
    //                                                 q7, qt_arr );
    // std::cout << "IK est. q: " << std::endl;
    // for(int i=0; i<4; i++){
    //     for (double val : q_est[i]) {
    //         std::cout << val << " ";  
    //     }
    //     std::cout << std::endl; 
    // }                                            

    // Initialization dq q_track M ev_diff
    MatrixXd qt_track(7,nbIter);
    MatrixXd dq_track(7,nbIter);
    MatrixXd J;
    MatrixXd J_geom;
    MatrixXd Me_ct(m,m);
    Tensor<double, 3> Me_track(m,m,nbIter);
    Tensor<double, 3> J_grad(m,n,n);
    MatrixXd Jm_t;
    MatrixXd M_diff(m,m);
    VectorXd vec_M_diff(21);
    VectorXd ev_t;
    MatrixXd ev_diff;

    std::cout << "Starting control loop-------------------------------------------------" << std::endl;
    // Main control loop //////////////////////////////////////////////////////////////////////////////
    for (int i = 0; i<nbIter; i++){
        VectorXd qt = vi.get_joint_positions(jointnames);
        qt_track.col(i) = qt;
        // Obtain the current analytical Jacobian, geom J and M 
        J = robot.pose_jacobian(qt);
        J_geom = geomJac(robot, J, qt, n); 
        Me_ct = J_geom*J_geom.transpose();

        // Record Me_ct into Me_track
        array<DenseIndex, 3> offset = {0, 0, i};
        array<DenseIndex, 3> extent = {m, m, 1};
        Tensor<double, 3> Me_ct_tensor = TensorMap<Tensor<double, 3>>(Me_ct.data(), 6, 6, 1);
        Me_track.slice(offset, extent) = Me_ct_tensor;
        J_grad = jacobianEst(geomJac, qt, n, robot);
        // Compute manipulability Jacobian (red to matrix)
        Jm_t = redManipulabilityJacobian(J_geom, J_grad);
        // std::cout<<"Jm_red: "<<std::endl<<Jm_red<<std::endl;

        // Compute distance to desired manipulybility 
        M_diff = logmap(Me_d, Me_ct); // 6x6
        vec_M_diff = spd2vec_vec(M_diff); // 21x1
        std::cout<<"M_diff: "<<std::endl<<M_diff<<std::endl;
        std::cout<<"vec_M_diff: "<<std::endl<<vec_M_diff<<std::endl;

        // Calculate eigenvalue of the current M (singular value of J_geom)
        BDCSVD<MatrixXd> singularsolver;
        Matrix<double, 6, 1> eigenvalue;
        eigenvalue = singularsolver.compute(J_geom).singularValues();
        ev_diff = jacobianEstVector(geomJac, qt, n, robot);

        // forward kinematic model
        DQ xt = robot.fkm(qt);
        
        // ++++++++++++++++++++QP Controller using osqp-eigen+++++++++++++++++++++++++
        constexpr double tolerance = 1e-4;
        c_float INFTY
        double K_qp = 2; 
        Matrix<c_float, 7, 7> H = Jm_t.transpose()*Jm_t;
        SparseMatrix<c_float> H_s;
        H_s = H.sparseView();
        std::cout<<"H: "<<std::endl<<H<<std::endl;
        std::cout<<"H_S : "<<std::endl<<H_s<<std::endl;
        // H_s.pruned(0.01); // set those who smaller than 0.01 as zero?
        Matrix<c_float, 1, 7> f = -K_qp* vec_M_diff.transpose()*Jm_t;
        // Inequality constraints:
        // 1. set min. allowed eigenvalue (min. ellipsoid axis length)
        double ev_min_r = 0.05;
        double ev_min_t = 0.01;
        Matrix<c_float, 6,1> v_max;
        v_max << ev_min_r, ev_min_r, ev_min_r, ev_min_t, ev_min_t, ev_min_t;
        Matrix<c_float, 6, 1> lb = v_max;
        
        Matrix<c_float, 6, 1> ub;

        OsqpEigen::Solver solver;
        //settings:
        // solver.settings()->setVerbosity(true); // print outptu or not
        solver.settings()->setAlpha(1.0); // ADMM relaxation parameter/step size/penalty parameter
        
        solver.data()->setNumberOfVariables(7);
        solver.data()->setNumberOfConstraints(6);
        solver.data()->setHessianMatrix(H_s);
        solver.data()->setGradient(f.transpose());
        // solver.data()->setLinearConstraintsMatrix(A_s);
        solver.data()->setLowerBound(lb);
        solver.data()->setUpperBound(ub);

        solver.initSolver();
        solver.solveProblem();
        // bool flag = solver.solveProblem() == OsqpEigen::ErrorExitFlag::NoError;
        // std::cout<<"No error: "<<flag<<std::endl;
        // Eigen::Matrix<c_float, 7, 1> expectedSolution;
        // expectedSolution << 0.3,  0.7;

        dq_track.col(i) = solver.getSolution();
        std::cout<<"Solution : "<<std::endl<<dq_track.col(i)<<std::endl;

        // bool converge = solver.getSolution().isApprox(expectedSolution, tolerance);
        // std::cout<<"Converged to desired solution: "<<converge<<std::endl;
        // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

        // Integrate to obtain q_out
        qt = qt + dq_track.col(i)*dt;
        // Send commands to the robot
        vi.set_joint_positions(jointnames,qt);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
    std::cout << "Control finished..." << std::endl;
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    
    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}


// test matlab
    // Quaterniond rotationQuaternion(0.1374, 0.72858, 0.62793, -0.23662);
    // Matrix3d rotationMatrix = rotationQuaternion.toRotationMatrix();
    // double roll, pitch, yaw;
    // roll = atan2(rotationMatrix(2, 1), rotationMatrix(2, 2));
    // pitch = atan2(-rotationMatrix(2, 0), sqrt(rotationMatrix(2, 1) * rotationMatrix(2, 1) + rotationMatrix(2, 2) * rotationMatrix(2, 2)));
    // yaw = atan2(rotationMatrix(1, 0), rotationMatrix(0, 0));
    // double scal = 180/pi;
    // std::cout<<"Euler angle in matlab: "<<roll*scal<<" "<<pitch*scal<<" "<<yaw*scal<<std::endl;

// Maximum joint ranges (deg): (q1..q7)
    //       -166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027  -166.0031
    // VectorXd q_min;
    // q_min << -2.8973,  -1.7628,  -2.8973,  -3.0718,  -2.8973   -0.0175  -2.8973;
    // //        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
    // VectorXd q_max;
    // q_max <<  2.8973,   1.7628,   2.8973,  -0.0698,   2.8973,   3.7525,   2.8973;

// DQ_PseudoinverseController controller(robot);
    // controller.set_gain(0.5);
    // controller.set_damping(0.05);
    // controller.set_control_objective(DQ_robotics::Translation);
    // controller.set_stability_threshold(0.00001);
    // DQ xdesired = 1 + E_*0.5*DQ(0, 0.2, 0.3, 0.3);
    // vi.set_object_pose("DesiredFrame", xdesired);
    // int i = 0;
    // while (not controller.system_reached_stable_region())
    // {
    //     VectorXd q = vi.get_joint_positions(jointnames);
    //     vi.set_object_pose("ReferenceFrame", robot->fkm(q));
    //     VectorXd u = controller.compute_setpoint_control_signal(q, vec4(xdesired.translation()));
    //     std::cout << "task error: " <<controller.get_last_error_signal().norm()
    //               <<" Iteration: "<<i<<std::endl;                  
    //     vi.set_joint_target_velocities(jointnames, u);
    //     vi.trigger_next_simulation_step();
    //     i++;
    // }

  // Update the base of the robot from CoppeliaSim
    // DQ new_base_robot = (robot.get_base_frame())*vi.getS_object_pose("Franka")*(1+0.5*E_*(0.107*k_));
    // robot.set_reference_frame(new_base_robot);
    // robot.set_base_frame(new_base_robot);


