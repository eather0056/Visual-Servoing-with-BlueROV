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
CMAKE_SOURCE_DIR = /home/tihan/catkin_ws/src/mavros_mavlink/mavlink

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/tihan/catkin_ws/build/mavlink

# Utility rule file for development.xml-v2.0.

# Include the progress variables for this target.
include CMakeFiles/development.xml-v2.0.dir/progress.make

CMakeFiles/development.xml-v2.0: development-v2.0-cxx-stamp


development-v2.0-cxx-stamp: /home/tihan/catkin_ws/src/mavros_mavlink/mavlink/message_definitions/v1.0/development.xml
development-v2.0-cxx-stamp: /home/tihan/catkin_ws/src/mavros_mavlink/mavlink/message_definitions/v1.0/common.xml
development-v2.0-cxx-stamp: /home/tihan/catkin_ws/src/mavros_mavlink/mavlink/pymavlink/tools/mavgen.py
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/tihan/catkin_ws/build/mavlink/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating development-v2.0-cxx-stamp"
	/usr/bin/env PYTHONPATH="/home/tihan/catkin_ws/src/mavros_mavlink/mavlink:/opt/ros/noetic/lib/python3/dist-packages" /usr/bin/python3.8 /home/tihan/catkin_ws/src/mavros_mavlink/mavlink/pymavlink/tools/mavgen.py --lang=C++11 --wire-protocol=2.0 --output=include/v2.0 /home/tihan/catkin_ws/src/mavros_mavlink/mavlink/message_definitions/v1.0/development.xml
	touch development-v2.0-cxx-stamp

development.xml-v2.0: CMakeFiles/development.xml-v2.0
development.xml-v2.0: development-v2.0-cxx-stamp
development.xml-v2.0: CMakeFiles/development.xml-v2.0.dir/build.make

.PHONY : development.xml-v2.0

# Rule to build all files generated by this target.
CMakeFiles/development.xml-v2.0.dir/build: development.xml-v2.0

.PHONY : CMakeFiles/development.xml-v2.0.dir/build

CMakeFiles/development.xml-v2.0.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/development.xml-v2.0.dir/cmake_clean.cmake
.PHONY : CMakeFiles/development.xml-v2.0.dir/clean

CMakeFiles/development.xml-v2.0.dir/depend:
	cd /home/tihan/catkin_ws/build/mavlink && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/tihan/catkin_ws/src/mavros_mavlink/mavlink /home/tihan/catkin_ws/src/mavros_mavlink/mavlink /home/tihan/catkin_ws/build/mavlink /home/tihan/catkin_ws/build/mavlink /home/tihan/catkin_ws/build/mavlink/CMakeFiles/development.xml-v2.0.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/development.xml-v2.0.dir/depend

