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

# Utility rule file for homework4_gencfg.

# Include the progress variables for this target.
include ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/progress.make

ugv_course/homework4/CMakeFiles/homework4_gencfg: devel/include/homework4/MarkerConfig.h
ugv_course/homework4/CMakeFiles/homework4_gencfg: devel/lib/python2.7/dist-packages/homework4/cfg/MarkerConfig.py

devel/include/homework4/MarkerConfig.h: ugv_course/homework4/cfg/Marker.cfg
devel/include/homework4/MarkerConfig.h: /opt/ros/indigo/share/dynamic_reconfigure/cmake/../templates/ConfigType.py.template
devel/include/homework4/MarkerConfig.h: /opt/ros/indigo/share/dynamic_reconfigure/cmake/../templates/ConfigType.h.template
	$(CMAKE_COMMAND) -E cmake_progress_report /home/student/ros/src/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating dynamic reconfigure files from cfg/Marker.cfg: /home/student/ros/src/devel/include/homework4/MarkerConfig.h /home/student/ros/src/devel/lib/python2.7/dist-packages/homework4/cfg/MarkerConfig.py"
	cd /home/student/ros/src/ugv_course/homework4 && ../../catkin_generated/env_cached.sh /home/student/ros/src/ugv_course/homework4/cfg/Marker.cfg /opt/ros/indigo/share/dynamic_reconfigure/cmake/.. /home/student/ros/src/devel/share/homework4 /home/student/ros/src/devel/include/homework4 /home/student/ros/src/devel/lib/python2.7/dist-packages/homework4

devel/share/homework4/docs/MarkerConfig.dox: devel/include/homework4/MarkerConfig.h

devel/share/homework4/docs/MarkerConfig-usage.dox: devel/include/homework4/MarkerConfig.h

devel/lib/python2.7/dist-packages/homework4/cfg/MarkerConfig.py: devel/include/homework4/MarkerConfig.h

devel/share/homework4/docs/MarkerConfig.wikidoc: devel/include/homework4/MarkerConfig.h

homework4_gencfg: ugv_course/homework4/CMakeFiles/homework4_gencfg
homework4_gencfg: devel/include/homework4/MarkerConfig.h
homework4_gencfg: devel/share/homework4/docs/MarkerConfig.dox
homework4_gencfg: devel/share/homework4/docs/MarkerConfig-usage.dox
homework4_gencfg: devel/lib/python2.7/dist-packages/homework4/cfg/MarkerConfig.py
homework4_gencfg: devel/share/homework4/docs/MarkerConfig.wikidoc
homework4_gencfg: ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/build.make
.PHONY : homework4_gencfg

# Rule to build all files generated by this target.
ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/build: homework4_gencfg
.PHONY : ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/build

ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/clean:
	cd /home/student/ros/src/ugv_course/homework4 && $(CMAKE_COMMAND) -P CMakeFiles/homework4_gencfg.dir/cmake_clean.cmake
.PHONY : ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/clean

ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/depend:
	cd /home/student/ros/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/student/ros/src /home/student/ros/src/ugv_course/homework4 /home/student/ros/src /home/student/ros/src/ugv_course/homework4 /home/student/ros/src/ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : ugv_course/homework4/CMakeFiles/homework4_gencfg.dir/depend

