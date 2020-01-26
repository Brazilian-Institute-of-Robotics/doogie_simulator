#include <string>

#include <grid_map_ros/grid_map_ros.hpp>
#include "doogie_rviz/maze_map.hpp"
#include "doogie_core/utils.hpp"
#include "doogie_core/maze_matrix_handle.hpp"

using namespace doogie_core;

namespace doogie_rviz {

MazeMap::MazeMap() 
  : maze_map_({"ground"})
  , is_to_update_map_(false) {
  maze_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("maze_map", 1);
  maze_obstacle_matrix_sub_ = nh_.subscribe("maze_obstacle_matrix", 1, &MazeMap::mazeObstacleMatrixCallback, this);
  doogie_position_sub_ = nh_.subscribe("doogie_position", 1, &MazeMap::doogiePositionCallback, this);
}

void MazeMap::loadParameters() {
  std::string maze_name;
  std::string log_prefix("doogie_rviz");
  ros::NodeHandle ph("~");

  DoogieUtils::getParameterHelper<float>(ph, log_prefix, "map_resolution", &map_resolution_, 0.01);
  DoogieUtils::getParameterHelper<float>(ph, log_prefix, "pub_rate", &pub_rate_, 10);
  DoogieUtils::getParameterHelper<float>(ph, log_prefix, "pivo_size", &pivo_size_, 0.012);
  DoogieUtils::getParameterHelper<std::string>(ph, log_prefix, "maze_name", &maze_name, "minus");
  
  number_of_rows_ = DoogieUtils::getNumberOfRows(maze_name);
  number_of_columns_ = DoogieUtils::getNumberOfColumns(maze_name);
  cell_size_ = DoogieUtils::getCellSize(maze_name);
}

void MazeMap::drawWall(grid_map::Position start, grid_map::Position end) {
  for (grid_map::LineIterator iterator(maze_map_, start, end); !iterator.isPastEnd(); ++iterator) {
    maze_map_.at("ground", *iterator) = 1.0;
  }
}

void MazeMap::init() {
  this->loadParameters();
  
  maze_map_.setFrameId("map");
  grid_map::Length maze_size(static_cast<float>(number_of_rows_) * (cell_size_ + pivo_size_) + pivo_size_,
                             static_cast<float>(number_of_columns_) * (cell_size_ + pivo_size_) + pivo_size_);
  maze_map_.setGeometry(maze_size, map_resolution_);

  float x_axis_length = maze_map_.getLength().x();
  float y_axis_length = maze_map_.getLength().y();

  // maze_map_.move(grid_map::Position(x_axis_length * -0.5, y_axis_length * -0.5));

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).", x_axis_length, y_axis_length,
           maze_map_.getSize()(0), maze_map_.getSize()(1));

  // Fills the entire map with the ground
  grid_map::Matrix& maze_map = this->maze_map_["ground"];
  for (grid_map::GridMapIterator iterator(maze_map_); !iterator.isPastEnd(); ++iterator) {
    const int i = iterator.getLinearIndex();
    maze_map(i) = 0;
  }

  /* Draw east, north, west and south walls respectively */
  this->drawWall(grid_map::Position(x_axis_length * -0.5, y_axis_length * 0.5), grid_map::Position(x_axis_length * 0.5, y_axis_length * 0.5));
  // this->drawWall(grid_map::Position(0, y_axis_length), grid_map::Position(x_axis_length, y_axis_length));
  // this->drawWall(grid_map::Position(x_axis_length, y_axis_length), grid_map::Position(x_axis_length, 0));
  // this->drawWall(grid_map::Position(x_axis_length, 0), grid_map::Position(0, 0));
}

void MazeMap::drawNorthWall(uint8_t row, uint8_t column) {

}

void MazeMap::drawSouthWall(uint8_t row, uint8_t column) {

}

void MazeMap::drawEastWall(uint8_t row, uint8_t column) {

}

void MazeMap::drawWestWall(uint8_t row, uint8_t column) {

}

void MazeMap::drawWalls(uint8_t row, uint8_t column, doogie_msgs::MazeCell cell_walls) {

}

void MazeMap::updateMap() {
  // if (!is_to_update_map_) return;

  grid_map::GridMapRosConverter::toOccupancyGrid(maze_map_, "ground", 0, 1.0, maze_map_grid_);
  maze_map_pub_.publish(maze_map_grid_);
}

void MazeMap::run() {
  ros::Rate rate(pub_rate_);
  
  while (ros::ok()) {
    this->updateMap();
    ros::spinOnce();
    rate.sleep();
  }
}

void MazeMap::doogiePositionCallback(const doogie_msgs::DoogiePositionConstPtr& doogie_position) {
  this->doogie_position_ = *doogie_position;
}

void MazeMap::mazeObstacleMatrixCallback(const doogie_msgs::MazeCellMultiArrayConstPtr& maze_obstacle_matrix) {
  cell_walls_ = MazeMatrixHandle::getMazeMatrixCell(*maze_obstacle_matrix,
                                                    doogie_position_.row, doogie_position_.column);
  is_to_update_map_ = true;
}

}  // namespace doogie_rviz
