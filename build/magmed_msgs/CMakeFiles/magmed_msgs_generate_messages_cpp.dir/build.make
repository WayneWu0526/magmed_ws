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

# Utility rule file for magmed_msgs_generate_messages_cpp.

# Include the progress variables for this target.
include magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/progress.make

magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/PFjoystick.h
magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/RoboJoints.h
magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/RoboStates.h
magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/TipAngle.h
magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h
magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h
magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/SelfCollisionCheck.h


/home/zhang/magmed_ws/devel/include/magmed_msgs/PFjoystick.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/zhang/magmed_ws/devel/include/magmed_msgs/PFjoystick.h: /home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PFjoystick.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PFjoystick.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating C++ code from magmed_msgs/PFjoystick.msg"
	cd /home/zhang/magmed_ws/src/magmed_msgs && /home/zhang/magmed_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg -Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p magmed_msgs -o /home/zhang/magmed_ws/devel/include/magmed_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/zhang/magmed_ws/devel/include/magmed_msgs/RoboJoints.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/zhang/magmed_ws/devel/include/magmed_msgs/RoboJoints.h: /home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/RoboJoints.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/RoboJoints.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating C++ code from magmed_msgs/RoboJoints.msg"
	cd /home/zhang/magmed_ws/src/magmed_msgs && /home/zhang/magmed_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg -Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p magmed_msgs -o /home/zhang/magmed_ws/devel/include/magmed_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/zhang/magmed_ws/devel/include/magmed_msgs/RoboStates.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/zhang/magmed_ws/devel/include/magmed_msgs/RoboStates.h: /home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/RoboStates.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Generating C++ code from magmed_msgs/RoboStates.msg"
	cd /home/zhang/magmed_ws/src/magmed_msgs && /home/zhang/magmed_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg -Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p magmed_msgs -o /home/zhang/magmed_ws/devel/include/magmed_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/zhang/magmed_ws/devel/include/magmed_msgs/TipAngle.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/zhang/magmed_ws/devel/include/magmed_msgs/TipAngle.h: /home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/TipAngle.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/TipAngle.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Generating C++ code from magmed_msgs/TipAngle.msg"
	cd /home/zhang/magmed_ws/src/magmed_msgs && /home/zhang/magmed_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg -Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p magmed_msgs -o /home/zhang/magmed_ws/devel/include/magmed_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /opt/ros/noetic/share/geometry_msgs/msg/Pose.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /opt/ros/noetic/share/geometry_msgs/msg/Point.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /opt/ros/noetic/share/geometry_msgs/msg/Twist.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /opt/ros/noetic/share/geometry_msgs/msg/Vector3.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /opt/ros/noetic/share/geometry_msgs/msg/Quaternion.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Generating C++ code from magmed_msgs/PoseTwist.msg"
	cd /home/zhang/magmed_ws/src/magmed_msgs && /home/zhang/magmed_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg -Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p magmed_msgs -o /home/zhang/magmed_ws/devel/include/magmed_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h: /home/zhang/magmed_ws/src/magmed_msgs/msg/MagCR.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h: /opt/ros/noetic/share/geometry_msgs/msg/Quaternion.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h: /opt/ros/noetic/share/geometry_msgs/msg/Pose.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h: /opt/ros/noetic/share/geometry_msgs/msg/Point.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h: /opt/ros/noetic/share/gencpp/msg.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Generating C++ code from magmed_msgs/MagCR.msg"
	cd /home/zhang/magmed_ws/src/magmed_msgs && /home/zhang/magmed_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/zhang/magmed_ws/src/magmed_msgs/msg/MagCR.msg -Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p magmed_msgs -o /home/zhang/magmed_ws/devel/include/magmed_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

/home/zhang/magmed_ws/devel/include/magmed_msgs/SelfCollisionCheck.h: /opt/ros/noetic/lib/gencpp/gen_cpp.py
/home/zhang/magmed_ws/devel/include/magmed_msgs/SelfCollisionCheck.h: /home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv
/home/zhang/magmed_ws/devel/include/magmed_msgs/SelfCollisionCheck.h: /home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/SelfCollisionCheck.h: /opt/ros/noetic/share/std_msgs/msg/Header.msg
/home/zhang/magmed_ws/devel/include/magmed_msgs/SelfCollisionCheck.h: /opt/ros/noetic/share/gencpp/msg.h.template
/home/zhang/magmed_ws/devel/include/magmed_msgs/SelfCollisionCheck.h: /opt/ros/noetic/share/gencpp/srv.h.template
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Generating C++ code from magmed_msgs/SelfCollisionCheck.srv"
	cd /home/zhang/magmed_ws/src/magmed_msgs && /home/zhang/magmed_ws/build/catkin_generated/env_cached.sh /usr/bin/python3 /opt/ros/noetic/share/gencpp/cmake/../../../lib/gencpp/gen_cpp.py /home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv -Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg -Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg -Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg -p magmed_msgs -o /home/zhang/magmed_ws/devel/include/magmed_msgs -e /opt/ros/noetic/share/gencpp/cmake/..

magmed_msgs_generate_messages_cpp: magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp
magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/PFjoystick.h
magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/RoboJoints.h
magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/RoboStates.h
magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/TipAngle.h
magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/PoseTwist.h
magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/MagCR.h
magmed_msgs_generate_messages_cpp: /home/zhang/magmed_ws/devel/include/magmed_msgs/SelfCollisionCheck.h
magmed_msgs_generate_messages_cpp: magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/build.make

.PHONY : magmed_msgs_generate_messages_cpp

# Rule to build all files generated by this target.
magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/build: magmed_msgs_generate_messages_cpp

.PHONY : magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/build

magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/clean:
	cd /home/zhang/magmed_ws/build/magmed_msgs && $(CMAKE_COMMAND) -P CMakeFiles/magmed_msgs_generate_messages_cpp.dir/cmake_clean.cmake
.PHONY : magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/clean

magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/depend:
	cd /home/zhang/magmed_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhang/magmed_ws/src /home/zhang/magmed_ws/src/magmed_msgs /home/zhang/magmed_ws/build /home/zhang/magmed_ws/build/magmed_msgs /home/zhang/magmed_ws/build/magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : magmed_msgs/CMakeFiles/magmed_msgs_generate_messages_cpp.dir/depend

