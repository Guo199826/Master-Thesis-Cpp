cmake_minimum_required(VERSION 3.6)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../dqrobotics_dependencies.cmake)
find_package (Threads REQUIRED)
find_package (osqp REQUIRED)
find_package (OsqpEigen REQUIRED)


project(joint_velocity_commands)


add_executable(joint_velocity_commands
    src/joint_velocity_commands.cpp
    src/jacobianEst.cpp
    src/jacobianEstVector.cpp
    src/geomJac.cpp
    src/test.cpp
    src/spdToVec.cpp
    src/FrankaRobot.cpp
    include/FrankaRobot.h
    include/manipulabilityJacobian.h
    src/logmap.cpp
    src/tmprod.cpp
    # src/symm2vec.cpp
    include/franka_analytical_ik-main/franka_ik_He.hpp
    src/dq2tfm.cpp
    )



TARGET_LINK_LIBRARIES(joint_velocity_commands
    pthread
    dqrobotics
    dqrobotics-interface-vrep
    osqp::osqp
    OsqpEigen::OsqpEigen
)

