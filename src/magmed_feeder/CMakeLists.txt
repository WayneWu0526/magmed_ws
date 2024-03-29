cmake_minimum_required(VERSION 3.0.2)
project(magmed_feeder)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
)

catkin_package(

)

add_executable(magrobo_feeder 
  src/magrobo_feeder.cpp)

add_dependencies(magrobo_feeder magmed_msgs_generate_messages_cpp)
add_dependencies(magrobo_feeder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

target_link_libraries(magrobo_feeder
  ${catkin_LIBRARIES}
)