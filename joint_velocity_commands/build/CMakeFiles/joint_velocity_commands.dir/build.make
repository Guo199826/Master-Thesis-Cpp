# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build

# Include any dependencies generated for this target.
include CMakeFiles/joint_velocity_commands.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/joint_velocity_commands.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/joint_velocity_commands.dir/flags.make

CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.o: ../src/manipulability_qp_controller.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/manipulability_qp_controller.cpp

CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/manipulability_qp_controller.cpp > CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/manipulability_qp_controller.cpp -o CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.o: ../src/jacobianEst.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/jacobianEst.cpp

CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/jacobianEst.cpp > CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/jacobianEst.cpp -o CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.o: ../src/jacobianEstVector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/jacobianEstVector.cpp

CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/jacobianEstVector.cpp > CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/jacobianEstVector.cpp -o CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.o: ../src/geomJac.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/geomJac.cpp

CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/geomJac.cpp > CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/geomJac.cpp -o CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.o: ../src/spdToVec.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/spdToVec.cpp

CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/spdToVec.cpp > CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/spdToVec.cpp -o CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.o: ../src/FrankaRobot.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/FrankaRobot.cpp

CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/FrankaRobot.cpp > CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/FrankaRobot.cpp -o CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.o: ../src/manipulabilityJacobian.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/manipulabilityJacobian.cpp

CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/manipulabilityJacobian.cpp > CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/manipulabilityJacobian.cpp -o CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.o: ../src/redManipulabilityJacobian.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/redManipulabilityJacobian.cpp

CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/redManipulabilityJacobian.cpp > CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/redManipulabilityJacobian.cpp -o CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.o: ../src/logmap.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/logmap.cpp

CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/logmap.cpp > CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/logmap.cpp -o CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.o: ../src/tmprod.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/tmprod.cpp

CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/tmprod.cpp > CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/tmprod.cpp -o CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.o: ../src/dq2tfm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/dq2tfm.cpp

CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/dq2tfm.cpp > CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/dq2tfm.cpp -o CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.o: ../src/readData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/readData.cpp

CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/readData.cpp > CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/readData.cpp -o CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.o: ../src/convert_csv2matrix.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/convert_csv2matrix.cpp

CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/convert_csv2matrix.cpp > CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/convert_csv2matrix.cpp -o CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.o: ../src/convert_matrix2csv.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.o -c /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/convert_matrix2csv.cpp

CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/convert_matrix2csv.cpp > CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/src/convert_matrix2csv.cpp -o CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.s

# Object files for target joint_velocity_commands
joint_velocity_commands_OBJECTS = \
"CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.o"

# External object files for target joint_velocity_commands
joint_velocity_commands_EXTERNAL_OBJECTS =

joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/manipulability_qp_controller.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/jacobianEst.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/jacobianEstVector.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/geomJac.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/spdToVec.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/FrankaRobot.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/manipulabilityJacobian.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/redManipulabilityJacobian.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/logmap.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/tmprod.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/dq2tfm.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/readData.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/convert_csv2matrix.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/convert_matrix2csv.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/build.make
joint_velocity_commands: /usr/local/lib/libOsqpEigen.so.0.8.1
joint_velocity_commands: /usr/local/lib/libosqp.so
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Linking CXX executable joint_velocity_commands"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joint_velocity_commands.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/joint_velocity_commands.dir/build: joint_velocity_commands

.PHONY : CMakeFiles/joint_velocity_commands.dir/build

CMakeFiles/joint_velocity_commands.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/joint_velocity_commands.dir/cmake_clean.cmake
.PHONY : CMakeFiles/joint_velocity_commands.dir/clean

CMakeFiles/joint_velocity_commands.dir/depend:
	cd /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build /home/guo/mani_qp_controller_vrep/mani_qp_controller_vrep/joint_velocity_commands/build/CMakeFiles/joint_velocity_commands.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/joint_velocity_commands.dir/depend

