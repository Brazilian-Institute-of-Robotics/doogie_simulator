cmake_minimum_required(VERSION 2.8.3)
project(doogie_gazebo)

SET(CATKIN_BUILD_DEPS
  roscpp
  sensor_msgs
  doogie_msgs
  message_filters
)

find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_DEPS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS ${CATKIN_BUILD_DEPS}
)


catkin_package()