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
CMAKE_SOURCE_DIR = /home/zhang/magmed_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/zhang/magmed_ws/build

# Include any dependencies generated for this target.
include magmed_controller/CMakeFiles/joyCtrlFeed.dir/depend.make

# Include the progress variables for this target.
include magmed_controller/CMakeFiles/joyCtrlFeed.dir/progress.make

# Include the compile flags for this target's objects.
include magmed_controller/CMakeFiles/joyCtrlFeed.dir/flags.make

magmed_controller/CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.o: magmed_controller/CMakeFiles/joyCtrlFeed.dir/flags.make
magmed_controller/CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.o: /home/zhang/magmed_ws/src/magmed_controller/src/joyCtrlFeed.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object magmed_controller/CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.o"
	cd /home/zhang/magmed_ws/build/magmed_controller && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.o -c /home/zhang/magmed_ws/src/magmed_controller/src/joyCtrlFeed.cpp

magmed_controller/CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.i"
	cd /home/zhang/magmed_ws/build/magmed_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhang/magmed_ws/src/magmed_controller/src/joyCtrlFeed.cpp > CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.i

magmed_controller/CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.s"
	cd /home/zhang/magmed_ws/build/magmed_controller && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhang/magmed_ws/src/magmed_controller/src/joyCtrlFeed.cpp -o CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.s

# Object files for target joyCtrlFeed
joyCtrlFeed_OBJECTS = \
"CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.o"

# External object files for target joyCtrlFeed
joyCtrlFeed_EXTERNAL_OBJECTS =

/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: magmed_controller/CMakeFiles/joyCtrlFeed.dir/src/joyCtrlFeed.cpp.o
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: magmed_controller/CMakeFiles/joyCtrlFeed.dir/build.make
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/libroscpp.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/librosconsole.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/librostime.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/libcpp_common.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /home/zhang/magmed_ws/devel/lib/libvelCtrlLib.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/libroscpp.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/librosconsole.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/librostime.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /opt/ros/noetic/lib/libcpp_common.so
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed: magmed_controller/CMakeFiles/joyCtrlFeed.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed"
	cd /home/zhang/magmed_ws/build/magmed_controller && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joyCtrlFeed.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
magmed_controller/CMakeFiles/joyCtrlFeed.dir/build: /home/zhang/magmed_ws/devel/lib/magmed_controller/joyCtrlFeed

.PHONY : magmed_controller/CMakeFiles/joyCtrlFeed.dir/build

magmed_controller/CMakeFiles/joyCtrlFeed.dir/clean:
	cd /home/zhang/magmed_ws/build/magmed_controller && $(CMAKE_COMMAND) -P CMakeFiles/joyCtrlFeed.dir/cmake_clean.cmake
.PHONY : magmed_controller/CMakeFiles/joyCtrlFeed.dir/clean

magmed_controller/CMakeFiles/joyCtrlFeed.dir/depend:
	cd /home/zhang/magmed_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhang/magmed_ws/src /home/zhang/magmed_ws/src/magmed_controller /home/zhang/magmed_ws/build /home/zhang/magmed_ws/build/magmed_controller /home/zhang/magmed_ws/build/magmed_controller/CMakeFiles/joyCtrlFeed.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : magmed_controller/CMakeFiles/joyCtrlFeed.dir/depend

