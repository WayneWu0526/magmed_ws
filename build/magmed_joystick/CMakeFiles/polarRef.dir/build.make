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
include magmed_joystick/CMakeFiles/polarRef.dir/depend.make

# Include the progress variables for this target.
include magmed_joystick/CMakeFiles/polarRef.dir/progress.make

# Include the compile flags for this target's objects.
include magmed_joystick/CMakeFiles/polarRef.dir/flags.make

magmed_joystick/CMakeFiles/polarRef.dir/src/polarRef.cpp.o: magmed_joystick/CMakeFiles/polarRef.dir/flags.make
magmed_joystick/CMakeFiles/polarRef.dir/src/polarRef.cpp.o: /home/zhang/magmed_ws/src/magmed_joystick/src/polarRef.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object magmed_joystick/CMakeFiles/polarRef.dir/src/polarRef.cpp.o"
	cd /home/zhang/magmed_ws/build/magmed_joystick && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/polarRef.dir/src/polarRef.cpp.o -c /home/zhang/magmed_ws/src/magmed_joystick/src/polarRef.cpp

magmed_joystick/CMakeFiles/polarRef.dir/src/polarRef.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/polarRef.dir/src/polarRef.cpp.i"
	cd /home/zhang/magmed_ws/build/magmed_joystick && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhang/magmed_ws/src/magmed_joystick/src/polarRef.cpp > CMakeFiles/polarRef.dir/src/polarRef.cpp.i

magmed_joystick/CMakeFiles/polarRef.dir/src/polarRef.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/polarRef.dir/src/polarRef.cpp.s"
	cd /home/zhang/magmed_ws/build/magmed_joystick && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhang/magmed_ws/src/magmed_joystick/src/polarRef.cpp -o CMakeFiles/polarRef.dir/src/polarRef.cpp.s

# Object files for target polarRef
polarRef_OBJECTS = \
"CMakeFiles/polarRef.dir/src/polarRef.cpp.o"

# External object files for target polarRef
polarRef_EXTERNAL_OBJECTS =

/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: magmed_joystick/CMakeFiles/polarRef.dir/src/polarRef.cpp.o
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: magmed_joystick/CMakeFiles/polarRef.dir/build.make
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/libserial.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/libroscpp.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/librosconsole.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/librostime.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /opt/ros/noetic/lib/libcpp_common.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: /home/zhang/magmed_ws/devel/lib/libPathFinderlib.so
/home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef: magmed_joystick/CMakeFiles/polarRef.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef"
	cd /home/zhang/magmed_ws/build/magmed_joystick && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/polarRef.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
magmed_joystick/CMakeFiles/polarRef.dir/build: /home/zhang/magmed_ws/devel/lib/magmed_joystick/polarRef

.PHONY : magmed_joystick/CMakeFiles/polarRef.dir/build

magmed_joystick/CMakeFiles/polarRef.dir/clean:
	cd /home/zhang/magmed_ws/build/magmed_joystick && $(CMAKE_COMMAND) -P CMakeFiles/polarRef.dir/cmake_clean.cmake
.PHONY : magmed_joystick/CMakeFiles/polarRef.dir/clean

magmed_joystick/CMakeFiles/polarRef.dir/depend:
	cd /home/zhang/magmed_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhang/magmed_ws/src /home/zhang/magmed_ws/src/magmed_joystick /home/zhang/magmed_ws/build /home/zhang/magmed_ws/build/magmed_joystick /home/zhang/magmed_ws/build/magmed_joystick/CMakeFiles/polarRef.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : magmed_joystick/CMakeFiles/polarRef.dir/depend
