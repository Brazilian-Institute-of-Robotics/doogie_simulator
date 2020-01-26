#ifndef DOOGIE_RVIZ_MAZE_MAP_HPP_
#define DOOGIE_RVIZ_MAZE_MAP_HPP_

#include <cstddef>

#include <ros/ros.h>
#include <grid_map_core/grid_map_core.hpp>
#include <nav_msgs/OccupancyGrid.h>

#include "doogie_msgs/MazeCellMultiArray.h"
#include "doogie_msgs/MazeCell.h"
#include "doogie_msgs/DoogiePosition.h"

namespace doogie_rviz {

class MazeMap {
 public:
  MazeMap();
  void init();
  inline void drawNorthWall(uint8_t row, uint8_t column);
  inline void drawSouthWall(uint8_t row, uint8_t column);
  inline void drawEastWall(uint8_t row, uint8_t column);
  inline void drawWestWall(uint8_t row, uint8_t column);
  void drawWalls(uint8_t row, uint8_t column, doogie_msgs::MazeCell cell_walls);
  void updateMap();
  void run();
 
 private:
  void loadParameters();
  void drawWall(grid_map::Position start, grid_map::Position end);
  void doogiePositionCallback(const doogie_msgs::DoogiePositionConstPtr& doogie_position);
  void mazeObstacleMatrixCallback(const doogie_msgs::MazeCellMultiArrayConstPtr& maze_obstacle_matrix);

  grid_map::GridMap maze_map_;
  ros::NodeHandle nh_;
  ros::Publisher maze_map_pub_;
  ros::Subscriber doogie_position_sub_;
  ros::Subscriber maze_obstacle_matrix_sub_;

  doogie_msgs::DoogiePosition doogie_position_;
  doogie_msgs::MazeCell cell_walls_;
  nav_msgs::OccupancyGrid maze_map_grid_;


  uint8_t number_of_rows_;
  uint8_t number_of_columns_;
  float cell_size_;
  float pivo_size_;
  float pub_rate_;
  float map_resolution_;

  bool is_to_update_map_;
};

} // namespace doogie_rviz

#endif  // DOOGIE_RVIZ_MAZE_MAP_HPP_
