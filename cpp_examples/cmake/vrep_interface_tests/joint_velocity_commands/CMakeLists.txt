cmake_minimum_required(VERSION 3.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../dqrobotics_dependencies.cmake)
find_package (Threads REQUIRED)
find_package (osqp REQUIRED)
find_package (OsqpEigen REQUIRED)


project(joint_velocity_commands)


add_executable(joint_velocity_commands
    # src/manipulability_qp_controller.cpp
    src/test_thread_controller.cpp
    src/jacobianEst.cpp
    src/jacobianEstVector.cpp
    src/geomJac.cpp
    # src/test.cpp
    src/spdToVec.cpp
    src/FrankaRobot.cpp
    include/FrankaRobot.h
    src/manipulabilityJacobian.cpp
    src/redManipulabilityJacobian.cpp
    src/logmap.cpp
    src/tmprod.cpp
    # include/franka_analytical_ik-main/franka_ik_He.hpp
    # src/dq2tfm.cpp
    # src/readData.cpp
    # src/matcreat.cpp
    )



TARGET_LINK_LIBRARIES(joint_velocity_commands
    pthread
    dqrobotics
    dqrobotics-interface-vrep
    osqp::osqp
    OsqpEigen::OsqpEigen
)

