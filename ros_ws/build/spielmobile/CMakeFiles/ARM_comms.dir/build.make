# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/pi/Documents/spielmobile/ros_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Documents/spielmobile/ros_ws/build

# Include any dependencies generated for this target.
include spielmobile/CMakeFiles/ARM_comms.dir/depend.make

# Include the progress variables for this target.
include spielmobile/CMakeFiles/ARM_comms.dir/progress.make

# Include the compile flags for this target's objects.
include spielmobile/CMakeFiles/ARM_comms.dir/flags.make

spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o: spielmobile/CMakeFiles/ARM_comms.dir/flags.make
spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o: /home/pi/Documents/spielmobile/ros_ws/src/spielmobile/ARM_comms.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Documents/spielmobile/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o"
	cd /home/pi/Documents/spielmobile/ros_ws/build/spielmobile && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o -c /home/pi/Documents/spielmobile/ros_ws/src/spielmobile/ARM_comms.cpp

spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ARM_comms.dir/ARM_comms.cpp.i"
	cd /home/pi/Documents/spielmobile/ros_ws/build/spielmobile && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Documents/spielmobile/ros_ws/src/spielmobile/ARM_comms.cpp > CMakeFiles/ARM_comms.dir/ARM_comms.cpp.i

spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ARM_comms.dir/ARM_comms.cpp.s"
	cd /home/pi/Documents/spielmobile/ros_ws/build/spielmobile && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Documents/spielmobile/ros_ws/src/spielmobile/ARM_comms.cpp -o CMakeFiles/ARM_comms.dir/ARM_comms.cpp.s

spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o.requires:

.PHONY : spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o.requires

spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o.provides: spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o.requires
	$(MAKE) -f spielmobile/CMakeFiles/ARM_comms.dir/build.make spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o.provides.build
.PHONY : spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o.provides

spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o.provides.build: spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o


# Object files for target ARM_comms
ARM_comms_OBJECTS = \
"CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o"

# External object files for target ARM_comms
ARM_comms_EXTERNAL_OBJECTS =

/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: spielmobile/CMakeFiles/ARM_comms.dir/build.make
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /opt/ros/kinetic/lib/libroscpp.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /opt/ros/kinetic/lib/librosconsole.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /opt/ros/kinetic/lib/librostime.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /opt/ros/kinetic/lib/libcpp_common.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms: spielmobile/CMakeFiles/ARM_comms.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Documents/spielmobile/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms"
	cd /home/pi/Documents/spielmobile/ros_ws/build/spielmobile && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ARM_comms.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
spielmobile/CMakeFiles/ARM_comms.dir/build: /home/pi/Documents/spielmobile/ros_ws/devel/lib/spielmobile/ARM_comms

.PHONY : spielmobile/CMakeFiles/ARM_comms.dir/build

spielmobile/CMakeFiles/ARM_comms.dir/requires: spielmobile/CMakeFiles/ARM_comms.dir/ARM_comms.cpp.o.requires

.PHONY : spielmobile/CMakeFiles/ARM_comms.dir/requires

spielmobile/CMakeFiles/ARM_comms.dir/clean:
	cd /home/pi/Documents/spielmobile/ros_ws/build/spielmobile && $(CMAKE_COMMAND) -P CMakeFiles/ARM_comms.dir/cmake_clean.cmake
.PHONY : spielmobile/CMakeFiles/ARM_comms.dir/clean

spielmobile/CMakeFiles/ARM_comms.dir/depend:
	cd /home/pi/Documents/spielmobile/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Documents/spielmobile/ros_ws/src /home/pi/Documents/spielmobile/ros_ws/src/spielmobile /home/pi/Documents/spielmobile/ros_ws/build /home/pi/Documents/spielmobile/ros_ws/build/spielmobile /home/pi/Documents/spielmobile/ros_ws/build/spielmobile/CMakeFiles/ARM_comms.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : spielmobile/CMakeFiles/ARM_comms.dir/depend

