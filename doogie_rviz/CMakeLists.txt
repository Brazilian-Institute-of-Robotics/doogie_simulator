cmake_minimum_required(VERSION 2.8.3)
project(doogie_rviz)

add_compile_options(-std=c++11)

set(DOOGIE_RVIZ_DEPS
  roscpp
  grid_map_core
  grid_map_ros
  nav_msgs
  doogie_msgs
  doogie_core
)

find_package(catkin REQUIRED COMPONENTS ${DOOGIE_RVIZ_DEPS})

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS ${DOOGIE_RVIZ_DEPS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(maze_map_node
  src/maze_map_node.cpp
  src/maze_map.cpp
)
add_dependencies(maze_map_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(maze_map_node ${catkin_LIBRARIES})
