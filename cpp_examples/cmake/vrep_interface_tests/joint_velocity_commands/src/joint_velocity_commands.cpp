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
// #include <dqrobotics/robots/FrankaEmikaPandaRobot.h>
#include "../include/FrankaRobot.h"
// #include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>
// #include <dqrobotics/robot_control/DQ_PseudoinverseController.h>
#include <thread>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "../include/dq2tfm.h"
// #include "../include/jacobianEst.h"
// #include "../include/geomJac.h"
// #include "osqp/osqp.h"
// #include "OsqpEigen/OsqpEigen.h"

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
    std::cout << "------base frame--------" << robot.get_base_frame()<<std::endl;
    std::cout << "------q lower limit-----" << robot.get_lower_q_limit()<<std::endl;

    // Update the base of the robot from CoppeliaSim
    // DQ new_base_robot = (robot.get_base_frame())*vi.get_object_pose("Franka")*(1+0.5*E_*(0.107*k_));
    // robot.set_reference_frame(new_base_robot);
    // robot.set_base_frame(new_base_robot);

    DQ base_frame = vi.get_object_pose("Franka_joint1");
    std::cout<<"bease frame in Vrep: "<<base_frame<<std::endl;
    robot.set_reference_frame(base_frame);
    robot.set_base_frame(base_frame);

    // Maximum joint ranges (deg): (q1..q7)
    //       -166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027  -166.0031
    // VectorXd q_min;
    // q_min << -2.8973,  -1.7628,  -2.8973,  -3.0718,  -2.8973   -0.0175  -2.8973;
    // //        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
    // VectorXd q_max;
    // q_max <<  2.8973,   1.7628,   2.8973,  -0.0698,   2.8973,   3.7525,   2.8973;
    // Set link number and joint angle
    int n = 7;
    VectorXd q_ (7);
    q_ << 1.1519, 0.3840, 0.2618, -1.5708, 0.0, 1.3963, 0.0 ; // validate with q_test in Matlab
    int m = 6; // Dimension of workspace

    // // Auxiliar variables
    // double dt = 1E-2;	// Time step
    int nbIter = 3; // Number of iterations (orig: 65)
    int nbData = 1; // no trajectory
    int t_all = nbIter*nbData;

    // Desired cartesian and manipulability trajectory

    // Initialization dq q_track M ev_diff
    MatrixXd q_track;

    // Define function handle for geomJac and pose_jacobian
    // std::function<MatrixXd(const DQ_SerialManipulator&, const MatrixXd &, 
    // const VectorXd&, const int)> fct_geomJac_ = geomJac;
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    
    
    // MatrixXd J_geom = geomJac(robot, J, q_, n); 
    // std::cout<<"J_geom: "<<std::endl<<J_geom<<std::endl;

    // test jacobianEstVector and jacobianEst
    // ev_diff: eigenvalue of Manipulability
    // MatrixXd J_sing = jacobianEstVector(geomJac, q_, n, robot); 
    // std::cout<<"JacobianEst for singular value: "<<std::endl<< J_sing <<std::endl;
    // Tensor<double, 3> J_jacobian = jacobianEst(geomJac, q_, n, robot_);
    std::cout << "Starting control loop..." << std::endl;
    VectorXd q_0 = vi.get_joint_positions(jointnames);
    DQ x_0 = robot.fkm(q_0);
    DQ x_0_t = x_0.translation();
    DQ x_0_r = x_0.rotation();

    Vector3d dq_t_vec = x_0.translation().vec3();
    Vector4d dq_r_vec = x_0.rotation().vec4();
    std::cout<<"dq t: "<<x_0.translation()<<std::endl;
    std::cout<<"dq r: "<<x_0_r<<std::endl;
    std::cout<<"dq r vec4: "<<dq_r_vec<<std::endl;

    Matrix4d tfm = dq2tfm(x_0);

    std::cout << "Joint positions q (at starting) is: \n"<< std::endl << vi.get_joint_positions(jointnames) << std::endl;
    std::cout<<"forward kinematics x_0_t: "<<std::endl<< x_0_t <<std::endl;
    std::cout<<"forward kinematics x_0_r: "<<std::endl<< x_0_r <<std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    // Send commands to the robot
    vi.set_joint_positions(jointnames,q_);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    std::cout << "Joint positions q (at starting) is set to be: \n"<< std::endl << vi.get_joint_positions(jointnames) << std::endl;

    // test geomJ
    MatrixXd J = robot.pose_jacobian(q_);
    DQ x = robot.fkm(q_);
    std::cout<<"J: "<<std::endl<<J<<std::endl;
    std::cout<<"forward kinematics x_t: "<<std::endl<< x <<std::endl;
    std::cout << "Joint positions q (at starting) is now: \n"<< std::endl << vi.get_joint_positions(jointnames) << std::endl;

    // Main control loop
    // for (int i = 0; i<nbIter; i++){
        // VectorXd q = vi.get_joint_positions(jointnames);
    //     vi.
        // Obtain the current analytical Jacobian (Dim: 8 * n)
        // MatrixXd J = robot.pose_jacobian(q);
        // // Send commands to the robot
        // vi.set_joint_positions(joint_names,q);
        //    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        // Integrate to obtain q_out
        // q = q + u;

    // }
    // std::cout << "Control finished..." << std::endl;
    ////////////////////////////////////////////////////////////////////////////////////////////////////////

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
    
    std::cout << "Stopping V-REP simulation..." << std::endl;
    vi.stop_simulation();
    vi.disconnect();
    return 0;
}
