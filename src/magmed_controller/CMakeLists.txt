# cmake_minimum_required(VERSION 3.0.2)
cmake_minimum_required(VERSION 3.5)
project(magmed_controller)

set(Eigen3_DIR /usr/lib/cmake/eigen3/)
set(qpOASES_DIR /usr/local/lib/cmake/qpOASES/)
set(NLopt_DIR /usr/local/lib/cmake/nlopt/)
# set(Ceres_DIR /usr/local/lib/cmake/Ceres/)
# set(Eigen3_DIR /usr/local/share/eigen3/cmake/)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  magmed_msgs
  Eigen3 REQUIRED
  qpOASES
  NLopt REQUIRED
  # Ceres REQUIRED
)

catkin_package(
)

include_directories(
include
  # /usr/include/eigen3
  ${catkin_INCLUDE_DIRS}
  # include/${PROJECT_NAME}
)
link_directories(
  /usr/local/lib
)
# link_directories(
#   /home/zhang/ModernRoboticsCpp/_install/lib
# )
## Declare a C++ library
add_library(velCtrlLib
  include/${PROJECT_NAME}/diffKine.h
  include/${PROJECT_NAME}/MSCRJacobi.h
  include/${PROJECT_NAME}/optCtrl.h
  include/${PROJECT_NAME}/velCtrlDef.h
  src/diffKine.cpp
  src/MSCRJacobi.cpp
  src/optCtrl.cpp
)

# add_executable(${PROJECT_NAME}_node src/magmed_controller_node.cpp)
add_executable(velocityController 
  src/velocityController.cpp
  src/MSCRJacobi.cpp)
add_executable(velCtrlNode
  src/velCtrlNode.cpp)
add_executable(joyCtrlFeed
  src/joyCtrlFeed.cpp)
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(velocityController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add magmed_msgs/MagPose.msg as a dependency of the executable
add_dependencies(velCtrlLib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(velCtrlNode magmed_msgs_generate_messages_cpp)
add_dependencies(velCtrlNode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
# add_dependencies(velCtrlNode ${PROJECT_NAME}_gencpp)

add_dependencies(joyCtrlFeed magmed_msgs_generate_messages_cpp)
add_dependencies(joyCtrlFeed ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_dependencies(velocityController magmed_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(getJacobianOfMCR
#   ${catkin_LIBRARIES}
# )
target_link_libraries(velCtrlLib
  ${catkin_LIBRARIES}
  qpOASES
  # ${NLopt_LIBRARIES}
  nlopt
  # Ceres::ceres
  ModernRoboticsCpp
)
target_link_libraries(velocityController
  ${catkin_LIBRARIES}
)
target_link_libraries(velCtrlNode
  ${catkin_LIBRARIES} 
  velCtrlLib
)
target_link_libraries(joyCtrlFeed
  ${catkin_LIBRARIES}
  velCtrlLib
)