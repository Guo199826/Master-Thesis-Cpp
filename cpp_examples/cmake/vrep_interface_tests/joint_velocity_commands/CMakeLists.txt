cmake_minimum_required(VERSION 3.1)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../dqrobotics_dependencies.cmake)
find_package (Threads REQUIRED)

project(joint_velocity_commands)


add_executable(joint_velocity_commands
    # src/joint_velocity_commands.cpp
    src/jacobianEst.cpp
    src/jacobianEstVector.cpp
    src/geomJac.cpp
    src/test.cpp
    src/spdToVec.cpp
    # src/manipulabilityJacobian.cpp
    # src/tmprod.cpp
    # src/logmap.cpp
    # src/symm2vec.cpp
    
    )


TARGET_LINK_LIBRARIES(joint_velocity_commands
    pthread
    dqrobotics
    dqrobotics-interface-vrep
)

