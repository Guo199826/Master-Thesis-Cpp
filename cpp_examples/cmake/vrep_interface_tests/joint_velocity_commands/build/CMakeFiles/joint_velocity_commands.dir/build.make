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
CMAKE_SOURCE_DIR = /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build

# Include any dependencies generated for this target.
include CMakeFiles/joint_velocity_commands.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/joint_velocity_commands.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/joint_velocity_commands.dir/flags.make

CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.o: ../src/joint_velocity_commands.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.o -c /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/joint_velocity_commands.cpp

CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/joint_velocity_commands.cpp > CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/joint_velocity_commands.cpp -o CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.o: ../src/jacobianest.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.o -c /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/jacobianest.cpp

CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/jacobianest.cpp > CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/jacobianest.cpp -o CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.s

CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.o: CMakeFiles/joint_velocity_commands.dir/flags.make
CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.o: ../src/geom_jacobian.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.o"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.o -c /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/geom_jacobian.cpp

CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.i"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/geom_jacobian.cpp > CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.i

CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.s"
	/usr/bin/g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/src/geom_jacobian.cpp -o CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.s

# Object files for target joint_velocity_commands
joint_velocity_commands_OBJECTS = \
"CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.o" \
"CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.o"

# External object files for target joint_velocity_commands
joint_velocity_commands_EXTERNAL_OBJECTS =

joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/joint_velocity_commands.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/jacobianest.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/src/geom_jacobian.cpp.o
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/build.make
joint_velocity_commands: CMakeFiles/joint_velocity_commands.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable joint_velocity_commands"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joint_velocity_commands.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/joint_velocity_commands.dir/build: joint_velocity_commands

.PHONY : CMakeFiles/joint_velocity_commands.dir/build

CMakeFiles/joint_velocity_commands.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/joint_velocity_commands.dir/cmake_clean.cmake
.PHONY : CMakeFiles/joint_velocity_commands.dir/clean

CMakeFiles/joint_velocity_commands.dir/depend:
	cd /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build /home/guo/master_thesis/Master-Thesis-Cpp/cpp-examples/cmake/vrep_interface_tests/joint_velocity_commands/build/CMakeFiles/joint_velocity_commands.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/joint_velocity_commands.dir/depend
