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
include ugv_course/homework3/CMakeFiles/test_diff.dir/depend.make

# Include the progress variables for this target.
include ugv_course/homework3/CMakeFiles/test_diff.dir/progress.make

# Include the compile flags for this target's objects.
include ugv_course/homework3/CMakeFiles/test_diff.dir/flags.make

ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o: ugv_course/homework3/CMakeFiles/test_diff.dir/flags.make
ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o: ugv_course/homework3/test/test_diff.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/student/ros/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o"
	cd /home/student/ros/src/ugv_course/homework3 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/test_diff.dir/test/test_diff.cpp.o -c /home/student/ros/src/ugv_course/homework3/test/test_diff.cpp

ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_diff.dir/test/test_diff.cpp.i"
	cd /home/student/ros/src/ugv_course/homework3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/student/ros/src/ugv_course/homework3/test/test_diff.cpp > CMakeFiles/test_diff.dir/test/test_diff.cpp.i

ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_diff.dir/test/test_diff.cpp.s"
	cd /home/student/ros/src/ugv_course/homework3 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/student/ros/src/ugv_course/homework3/test/test_diff.cpp -o CMakeFiles/test_diff.dir/test/test_diff.cpp.s

ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o.requires:
.PHONY : ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o.requires

ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o.provides: ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o.requires
	$(MAKE) -f ugv_course/homework3/CMakeFiles/test_diff.dir/build.make ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o.provides.build
.PHONY : ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o.provides

ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o.provides.build: ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o

# Object files for target test_diff
test_diff_OBJECTS = \
"CMakeFiles/test_diff.dir/test/test_diff.cpp.o"

# External object files for target test_diff
test_diff_EXTERNAL_OBJECTS =

devel/lib/homework3/test_diff: ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o
devel/lib/homework3/test_diff: ugv_course/homework3/CMakeFiles/test_diff.dir/build.make
devel/lib/homework3/test_diff: /opt/ros/indigo/lib/libroscpp.so
devel/lib/homework3/test_diff: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/homework3/test_diff: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/homework3/test_diff: /opt/ros/indigo/lib/librosconsole.so
devel/lib/homework3/test_diff: /opt/ros/indigo/lib/librosconsole_log4cxx.so
devel/lib/homework3/test_diff: /opt/ros/indigo/lib/librosconsole_backend_interface.so
devel/lib/homework3/test_diff: /usr/lib/liblog4cxx.so
devel/lib/homework3/test_diff: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/homework3/test_diff: /opt/ros/indigo/lib/libroscpp_serialization.so
devel/lib/homework3/test_diff: /opt/ros/indigo/lib/librostime.so
devel/lib/homework3/test_diff: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/homework3/test_diff: /opt/ros/indigo/lib/libxmlrpcpp.so
devel/lib/homework3/test_diff: /opt/ros/indigo/lib/libcpp_common.so
devel/lib/homework3/test_diff: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/homework3/test_diff: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/homework3/test_diff: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/homework3/test_diff: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/homework3/test_diff: gtest/libgtest.so
devel/lib/homework3/test_diff: ugv_course/homework3/CMakeFiles/test_diff.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../devel/lib/homework3/test_diff"
	cd /home/student/ros/src/ugv_course/homework3 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_diff.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
ugv_course/homework3/CMakeFiles/test_diff.dir/build: devel/lib/homework3/test_diff
.PHONY : ugv_course/homework3/CMakeFiles/test_diff.dir/build

ugv_course/homework3/CMakeFiles/test_diff.dir/requires: ugv_course/homework3/CMakeFiles/test_diff.dir/test/test_diff.cpp.o.requires
.PHONY : ugv_course/homework3/CMakeFiles/test_diff.dir/requires

ugv_course/homework3/CMakeFiles/test_diff.dir/clean:
	cd /home/student/ros/src/ugv_course/homework3 && $(CMAKE_COMMAND) -P CMakeFiles/test_diff.dir/cmake_clean.cmake
.PHONY : ugv_course/homework3/CMakeFiles/test_diff.dir/clean

ugv_course/homework3/CMakeFiles/test_diff.dir/depend:
	cd /home/student/ros/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/ros/src /home/student/ros/src/ugv_course/homework3 /home/student/ros/src /home/student/ros/src/ugv_course/homework3 /home/student/ros/src/ugv_course/homework3/CMakeFiles/test_diff.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ugv_course/homework3/CMakeFiles/test_diff.dir/depend

