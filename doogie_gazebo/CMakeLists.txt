cmake_minimum_required(VERSION 2.8.3)
project(doogie_gazebo)

add_compile_options(-std=c++11)

SET(CATKIN_BUILD_DEPS
  roscpp
  sensor_msgs
  doogie_msgs
  message_filters
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPS})
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS ${CATKIN_BUILD_DEPS}
  DEPENDS Boost
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(ir_sensor_data_acc_node
  src/main.cpp  
  src/ir_sensor_data_acc.cpp
)
target_link_libraries(ir_sensor_data_acc_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
