# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(FATAL_ERROR "Could not find messages which '/home/zhang/magmed_ws/src/magmed_srv/srv/SelfCollisionCheck.srv' depends on. Did you forget to specify generate_messages(DEPENDENCIES ...)?
Cannot locate message [RoboJoints]: unknown package [magmed_srv] on search path [{'std_msgs': ['/opt/ros/noetic/share/std_msgs/cmake/../msg'], 'magmed_msgs': ['/home/zhang/magmed_ws/src/magmed_msgs/msg'], 'geometry_msgs': ['/opt/ros/noetic/share/geometry_msgs/cmake/../msg']}]")
message(STATUS "magmed_srv: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Imagmed_msgs:/home/zhang/magmed_ws/src/magmed_msgs/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(magmed_srv_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_cpp(magmed_srv
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_srv
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(magmed_srv_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(magmed_srv_generate_messages magmed_srv_generate_messages_cpp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(magmed_srv_gencpp)
add_dependencies(magmed_srv_gencpp magmed_srv_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_srv_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_eus(magmed_srv
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_srv
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(magmed_srv_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(magmed_srv_generate_messages magmed_srv_generate_messages_eus)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(magmed_srv_geneus)
add_dependencies(magmed_srv_geneus magmed_srv_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_srv_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_lisp(magmed_srv
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_srv
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(magmed_srv_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(magmed_srv_generate_messages magmed_srv_generate_messages_lisp)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(magmed_srv_genlisp)
add_dependencies(magmed_srv_genlisp magmed_srv_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_srv_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_nodejs(magmed_srv
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_srv
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(magmed_srv_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(magmed_srv_generate_messages magmed_srv_generate_messages_nodejs)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(magmed_srv_gennodejs)
add_dependencies(magmed_srv_gennodejs magmed_srv_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_srv_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services

### Generating Module File
_generate_module_py(magmed_srv
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_srv
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(magmed_srv_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(magmed_srv_generate_messages magmed_srv_generate_messages_py)

# add dependencies to all check dependencies targets

# target for backward compatibility
add_custom_target(magmed_srv_genpy)
add_dependencies(magmed_srv_genpy magmed_srv_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS magmed_srv_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_srv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/magmed_srv
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(magmed_srv_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET magmed_msgs_generate_messages_cpp)
  add_dependencies(magmed_srv_generate_messages_cpp magmed_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_srv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/magmed_srv
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(magmed_srv_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET magmed_msgs_generate_messages_eus)
  add_dependencies(magmed_srv_generate_messages_eus magmed_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_srv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/magmed_srv
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(magmed_srv_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET magmed_msgs_generate_messages_lisp)
  add_dependencies(magmed_srv_generate_messages_lisp magmed_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_srv)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/magmed_srv
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(magmed_srv_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET magmed_msgs_generate_messages_nodejs)
  add_dependencies(magmed_srv_generate_messages_nodejs magmed_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_srv)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_srv\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/magmed_srv
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(magmed_srv_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET magmed_msgs_generate_messages_py)
  add_dependencies(magmed_srv_generate_messages_py magmed_msgs_generate_messages_py)
endif()
