# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/student/ros/src/ugv_course/gps_sim_project

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/ros/src/ugv_course/gps_sim_project

# Utility rule file for clean_test_results_gps_sim_project.

# Include the progress variables for this target.
include CMakeFiles/clean_test_results_gps_sim_project.dir/progress.make

CMakeFiles/clean_test_results_gps_sim_project:
	/usr/bin/python /opt/ros/indigo/share/catkin/cmake/test/remove_test_results.py /home/student/ros/src/ugv_course/gps_sim_project/test_results/gps_sim_project

clean_test_results_gps_sim_project: CMakeFiles/clean_test_results_gps_sim_project
clean_test_results_gps_sim_project: CMakeFiles/clean_test_results_gps_sim_project.dir/build.make
.PHONY : clean_test_results_gps_sim_project

# Rule to build all files generated by this target.
CMakeFiles/clean_test_results_gps_sim_project.dir/build: clean_test_results_gps_sim_project
.PHONY : CMakeFiles/clean_test_results_gps_sim_project.dir/build

CMakeFiles/clean_test_results_gps_sim_project.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/clean_test_results_gps_sim_project.dir/cmake_clean.cmake
.PHONY : CMakeFiles/clean_test_results_gps_sim_project.dir/clean

CMakeFiles/clean_test_results_gps_sim_project.dir/depend:
	cd /home/student/ros/src/ugv_course/gps_sim_project && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/ros/src/ugv_course/gps_sim_project /home/student/ros/src/ugv_course/gps_sim_project /home/student/ros/src/ugv_course/gps_sim_project /home/student/ros/src/ugv_course/gps_sim_project /home/student/ros/src/ugv_course/gps_sim_project/CMakeFiles/clean_test_results_gps_sim_project.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/clean_test_results_gps_sim_project.dir/depend

