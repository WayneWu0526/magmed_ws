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
include magmed_camera/CMakeFiles/demo01_camera_connect.dir/depend.make

# Include the progress variables for this target.
include magmed_camera/CMakeFiles/demo01_camera_connect.dir/progress.make

# Include the compile flags for this target's objects.
include magmed_camera/CMakeFiles/demo01_camera_connect.dir/flags.make

magmed_camera/CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.o: magmed_camera/CMakeFiles/demo01_camera_connect.dir/flags.make
magmed_camera/CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.o: /home/zhang/magmed_ws/src/magmed_camera/src/demo01_camera_connect.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object magmed_camera/CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.o"
	cd /home/zhang/magmed_ws/build/magmed_camera && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.o -c /home/zhang/magmed_ws/src/magmed_camera/src/demo01_camera_connect.cpp

magmed_camera/CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.i"
	cd /home/zhang/magmed_ws/build/magmed_camera && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/zhang/magmed_ws/src/magmed_camera/src/demo01_camera_connect.cpp > CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.i

magmed_camera/CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.s"
	cd /home/zhang/magmed_ws/build/magmed_camera && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/zhang/magmed_ws/src/magmed_camera/src/demo01_camera_connect.cpp -o CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.s

# Object files for target demo01_camera_connect
demo01_camera_connect_OBJECTS = \
"CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.o"

# External object files for target demo01_camera_connect
demo01_camera_connect_EXTERNAL_OBJECTS =

/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: magmed_camera/CMakeFiles/demo01_camera_connect.dir/src/demo01_camera_connect.cpp.o
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: magmed_camera/CMakeFiles/demo01_camera_connect.dir/build.make
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /opt/ros/noetic/lib/libroscpp.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libboost_chrono.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /opt/ros/noetic/lib/librosconsole.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /opt/ros/noetic/lib/librosconsole_log4cxx.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /opt/ros/noetic/lib/librosconsole_backend_interface.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libboost_regex.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /opt/ros/noetic/lib/libxmlrpcpp.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /opt/ros/noetic/lib/libroscpp_serialization.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /opt/ros/noetic/lib/librostime.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libboost_date_time.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /opt/ros/noetic/lib/libcpp_common.so
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so.0.4
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_aruco.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_bgsegm.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_bioinspired.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_ccalib.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_dnn_objdetect.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_dnn_superres.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_dpm.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_face.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_freetype.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_fuzzy.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_hdf.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_hfs.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_img_hash.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_line_descriptor.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_quality.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_reg.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_rgbd.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_saliency.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_shape.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_stereo.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_structured_light.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_surface_matching.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_tracking.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_viz.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_xobjdetect.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_xphoto.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_datasets.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_plot.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_text.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_dnn.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_phase_unwrapping.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_optflow.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_ximgproc.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_video.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_videoio.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_imgcodecs.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: /usr/lib/x86_64-linux-gnu/libopencv_core.so.4.2.0
/home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect: magmed_camera/CMakeFiles/demo01_camera_connect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/zhang/magmed_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect"
	cd /home/zhang/magmed_ws/build/magmed_camera && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/demo01_camera_connect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
magmed_camera/CMakeFiles/demo01_camera_connect.dir/build: /home/zhang/magmed_ws/devel/lib/magmed_camera/demo01_camera_connect

.PHONY : magmed_camera/CMakeFiles/demo01_camera_connect.dir/build

magmed_camera/CMakeFiles/demo01_camera_connect.dir/clean:
	cd /home/zhang/magmed_ws/build/magmed_camera && $(CMAKE_COMMAND) -P CMakeFiles/demo01_camera_connect.dir/cmake_clean.cmake
.PHONY : magmed_camera/CMakeFiles/demo01_camera_connect.dir/clean

magmed_camera/CMakeFiles/demo01_camera_connect.dir/depend:
	cd /home/zhang/magmed_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/zhang/magmed_ws/src /home/zhang/magmed_ws/src/magmed_camera /home/zhang/magmed_ws/build /home/zhang/magmed_ws/build/magmed_camera /home/zhang/magmed_ws/build/magmed_camera/CMakeFiles/demo01_camera_connect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : magmed_camera/CMakeFiles/demo01_camera_connect.dir/depend

