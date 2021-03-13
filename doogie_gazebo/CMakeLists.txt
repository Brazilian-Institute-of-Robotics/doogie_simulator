cmake_minimum_required(VERSION 2.8.3)
project(doogie_gazebo)

add_compile_options(-std=c++11)

SET(CATKIN_BUILD_DEPS
  angles
  doogie_msgs
  doogie_localization
  gazebo_msgs
  geometry_msgs
  message_filters
  nav_msgs
  pluginlib
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPS})
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ground_truth
  CATKIN_DEPENDS ${CATKIN_BUILD_DEPS}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_library(gazebo_ground_truth
  src/ground_truth.cpp
)
target_link_libraries(gazebo_ground_truth
  ${catkin_LIBRARIES}
)

add_executable(ir_sensor_data_acc_node
  src/ir_sensor_data_acc_node.cpp  
  src/ir_sensor_data_acc.cpp
)
target_link_libraries(ir_sensor_data_acc_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
