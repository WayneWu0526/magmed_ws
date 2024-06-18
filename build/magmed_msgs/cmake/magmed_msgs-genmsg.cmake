# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "magmed_msgs: 5 messages, 1 services")

set(MSG_I_FLAGS "-Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(magmed_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg" NAME_WE)
add_custom_target(_magmed_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "magmed_msgs" "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg" NAME_WE)
add_custom_target(_magmed_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "magmed_msgs" "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg" NAME_WE)
add_custom_target(_magmed_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "magmed_msgs" "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg" ""
)

get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg" NAME_WE)
add_custom_target(_magmed_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "magmed_msgs" "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg" NAME_WE)
add_custom_target(_magmed_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "magmed_msgs" "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg" "geometry_msgs/Pose:geometry_msgs/Twist:geometry_msgs/Vector3:geometry_msgs/Point:std_msgs/Header:geometry_msgs/Quaternion"
)

get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv" NAME_WE)
add_custom_target(_magmed_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "magmed_msgs" "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv" "magmed_msgs/RoboJoints:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs
)
_generate_msg_cpp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs
)
_generate_msg_cpp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs
)
_generate_msg_cpp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs
)
_generate_msg_cpp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs
)

### Generating Services
_generate_srv_cpp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv"
  "${MSG_I_FLAGS}"
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs
)

### Generating Module File
_generate_module_cpp(magmed_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(magmed_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(magmed_msgs_generate_messages magmed_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_cpp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_cpp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_cpp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_cpp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_cpp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_cpp _magmed_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(magmed_msgs_gencpp)
add_dependencies(magmed_msgs_gencpp magmed_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs
)
_generate_msg_eus(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs
)
_generate_msg_eus(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs
)
_generate_msg_eus(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs
)
_generate_msg_eus(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs
)

### Generating Services
_generate_srv_eus(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv"
  "${MSG_I_FLAGS}"
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs
)

### Generating Module File
_generate_module_eus(magmed_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(magmed_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(magmed_msgs_generate_messages magmed_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_eus _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_eus _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_eus _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_eus _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_eus _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_eus _magmed_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(magmed_msgs_geneus)
add_dependencies(magmed_msgs_geneus magmed_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs
)
_generate_msg_lisp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs
)
_generate_msg_lisp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs
)
_generate_msg_lisp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs
)
_generate_msg_lisp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs
)

### Generating Services
_generate_srv_lisp(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv"
  "${MSG_I_FLAGS}"
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs
)

### Generating Module File
_generate_module_lisp(magmed_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(magmed_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(magmed_msgs_generate_messages magmed_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_lisp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_lisp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_lisp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_lisp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_lisp _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_lisp _magmed_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(magmed_msgs_genlisp)
add_dependencies(magmed_msgs_genlisp magmed_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs
)
_generate_msg_nodejs(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs
)
_generate_msg_nodejs(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs
)
_generate_msg_nodejs(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs
)
_generate_msg_nodejs(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs
)

### Generating Services
_generate_srv_nodejs(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv"
  "${MSG_I_FLAGS}"
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs
)

### Generating Module File
_generate_module_nodejs(magmed_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(magmed_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(magmed_msgs_generate_messages magmed_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_nodejs _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_nodejs _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_nodejs _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_nodejs _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_nodejs _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_nodejs _magmed_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(magmed_msgs_gennodejs)
add_dependencies(magmed_msgs_gennodejs magmed_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs
)
_generate_msg_py(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs
)
_generate_msg_py(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs
)
_generate_msg_py(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs
)
_generate_msg_py(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs
)

### Generating Services
_generate_srv_py(magmed_msgs
  "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv"
  "${MSG_I_FLAGS}"
  "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs
)

### Generating Module File
_generate_module_py(magmed_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(magmed_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(magmed_msgs_generate_messages magmed_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PFjoystick.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_py _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboJoints.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_py _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/RoboStates.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_py _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/TipAngle.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_py _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/msg/PoseTwist.msg" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_py _magmed_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/zhang/magmed_ws/src/magmed_msgs/srv/SelfCollisionCheck.srv" NAME_WE)
add_dependencies(magmed_msgs_generate_messages_py _magmed_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(magmed_msgs_genpy)
add_dependencies(magmed_msgs_genpy magmed_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(magmed_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(magmed_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(magmed_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(magmed_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(magmed_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(magmed_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(magmed_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(magmed_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(magmed_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(magmed_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
