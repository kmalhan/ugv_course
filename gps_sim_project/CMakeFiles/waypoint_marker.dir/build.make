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
CMAKE_SOURCE_DIR = /home/student/ros/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/student/ros/src

# Include any dependencies generated for this target.
include ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/depend.make

# Include the progress variables for this target.
include ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/progress.make

# Include the compile flags for this target's objects.
include ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/flags.make

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o: ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/flags.make
ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o: ugv_course/gps_sim_project/src/waypoint_marker.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/student/ros/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o"
	cd /home/student/ros/src/ugv_course/gps_sim_project && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o -c /home/student/ros/src/ugv_course/gps_sim_project/src/waypoint_marker.cpp

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.i"
	cd /home/student/ros/src/ugv_course/gps_sim_project && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/student/ros/src/ugv_course/gps_sim_project/src/waypoint_marker.cpp > CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.i

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.s"
	cd /home/student/ros/src/ugv_course/gps_sim_project && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/student/ros/src/ugv_course/gps_sim_project/src/waypoint_marker.cpp -o CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.s

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o.requires:
.PHONY : ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o.requires

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o.provides: ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o.requires
	$(MAKE) -f ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/build.make ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o.provides.build
.PHONY : ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o.provides

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o.provides.build: ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o

# Object files for target waypoint_marker
waypoint_marker_OBJECTS = \
"CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o"

# External object files for target waypoint_marker
waypoint_marker_EXTERNAL_OBJECTS =

devel/lib/gps_sim_project/waypoint_marker: ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o
devel/lib/gps_sim_project/waypoint_marker: ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/build.make
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libtf.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libtf2_ros.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libactionlib.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libmessage_filters.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libroscpp.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libxmlrpcpp.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libtf2.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libroscpp_serialization.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/librosconsole.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/librosconsole_log4cxx.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/librosconsole_backend_interface.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/liblog4cxx.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/librostime.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/gps_sim_project/waypoint_marker: /opt/ros/indigo/lib/libcpp_common.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/gps_sim_project/waypoint_marker: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/gps_sim_project/waypoint_marker: ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../devel/lib/gps_sim_project/waypoint_marker"
	cd /home/student/ros/src/ugv_course/gps_sim_project && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/waypoint_marker.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/build: devel/lib/gps_sim_project/waypoint_marker
.PHONY : ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/build

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/requires: ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/src/waypoint_marker.cpp.o.requires
.PHONY : ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/requires

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/clean:
	cd /home/student/ros/src/ugv_course/gps_sim_project && $(CMAKE_COMMAND) -P CMakeFiles/waypoint_marker.dir/cmake_clean.cmake
.PHONY : ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/clean

ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/depend:
	cd /home/student/ros/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/ros/src /home/student/ros/src/ugv_course/gps_sim_project /home/student/ros/src /home/student/ros/src/ugv_course/gps_sim_project /home/student/ros/src/ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ugv_course/gps_sim_project/CMakeFiles/waypoint_marker.dir/depend

