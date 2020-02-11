#include <ros/ros.h>
#include "doogie_rviz/maze_map.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "maze_map_node");

  doogie_rviz::MazeMap maze_map;

  try {
    maze_map.init();
    maze_map.run();
  } catch (std::runtime_error &error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  return 0;
}
