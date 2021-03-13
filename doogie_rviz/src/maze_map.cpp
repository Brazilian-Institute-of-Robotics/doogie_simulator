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
  maze_obstacle_matrix_sub_ = nh_.subscribe("maze_matrix", 1, &MazeMap::mazeObstacleMatrixCallback, this);
  doogie_pose_sub_ = nh_.subscribe("doogie_pose", 1, &MazeMap::doogiePoseCallback, this);
}

void MazeMap::loadParameters() {
  std::string maze_name;
  std::string log_prefix("doogie_rviz");
  ros::NodeHandle ph("~");

  DoogieUtils::getParameterHelper<float>(ph, log_prefix, "map_resolution", &map_resolution_, 0.01);
  DoogieUtils::getParameterHelper<float>(ph, log_prefix, "pub_rate", &pub_rate_, 10);
  DoogieUtils::getParameterHelper<float>(ph, log_prefix, "pivo_dim", &pivo_dim_, 0.012);
  DoogieUtils::getParameterHelper<std::string>(ph, log_prefix, "maze_name", &maze_name, "minus");
  
  number_of_rows_ = DoogieUtils::getNumberOfRows(maze_name);
  number_of_columns_ = DoogieUtils::getNumberOfColumns(maze_name);
  cell_size_ = DoogieUtils::getCellSize(maze_name);

  wall_size_ = cell_size_ + (2 * pivo_dim_); 
}

void MazeMap::drawWall(grid_map::Position start, grid_map::Position end) {
  for (grid_map::LineIterator iterator(maze_map_, start, end); !iterator.isPastEnd(); ++iterator) {
    maze_map_.at("ground", *iterator) = 1.0;
  }
}

grid_map::Position MazeMap::computeStartPoint(uint8_t row, uint8_t column) {
  float x_zero = static_cast<float>(column) * (cell_size_ + pivo_dim_);
  float y_zero = static_cast<float>(row) * (cell_size_ + pivo_dim_);
  
  return grid_map::Position(x_zero, y_zero);
}

void MazeMap::init() {
  this->loadParameters();
  
  maze_map_.setFrameId("map");
  grid_map::Length maze_size(static_cast<float>(number_of_rows_) * (cell_size_ + pivo_dim_) + pivo_dim_,
                             static_cast<float>(number_of_columns_) * (cell_size_ + pivo_dim_) + pivo_dim_);
  maze_map_.setGeometry(maze_size, map_resolution_);

  float x_axis_length = maze_map_.getLength().x();
  float y_axis_length = maze_map_.getLength().y();

  maze_map_.setPosition(grid_map::Position(x_axis_length * 0.5, y_axis_length * 0.5));

  ROS_INFO("Created map with size %f x %f m (%i x %i cells).", x_axis_length, y_axis_length,
           maze_map_.getSize()(0), maze_map_.getSize()(1));

  // Fills the entire map with the ground
  grid_map::Matrix& maze_map = this->maze_map_["ground"];
  for (grid_map::GridMapIterator iterator(maze_map_); !iterator.isPastEnd(); ++iterator) {
    const int i = iterator.getLinearIndex();
    maze_map(i) = 0;
  }

  /* Draw east, north, west and south walls respectively */
  this->drawWall(grid_map::Position(0, 0), grid_map::Position(0, y_axis_length));
  this->drawWall(grid_map::Position(0, y_axis_length), grid_map::Position(x_axis_length, y_axis_length));
  this->drawWall(grid_map::Position(x_axis_length, y_axis_length), grid_map::Position(x_axis_length, 0));
  this->drawWall(grid_map::Position(x_axis_length, 0), grid_map::Position(0, 0));
}

void MazeMap::drawNorthWall(uint8_t row, uint8_t column) {
  grid_map::Position start_point = this->computeStartPoint(row + 1, column);
  this->drawWall(start_point, grid_map::Position(start_point.x() + wall_size_, start_point.y()));
}

void MazeMap::drawSouthWall(uint8_t row, uint8_t column) {
  grid_map::Position start_point = this->computeStartPoint(row, column);
  this->drawWall(start_point, grid_map::Position(start_point.x() + wall_size_, start_point.y()));
}

void MazeMap::drawEastWall(uint8_t row, uint8_t column) {
  grid_map::Position start_point = this->computeStartPoint(row, column + 1);
  this->drawWall(start_point, grid_map::Position(start_point.x(), start_point.y() + wall_size_));
}

void MazeMap::drawWestWall(uint8_t row, uint8_t column) {
  grid_map::Position start_point = this->computeStartPoint(row, column);
  this->drawWall(start_point, grid_map::Position(start_point.x(), start_point.y() + wall_size_));
}

void MazeMap::drawWalls(uint8_t row, uint8_t column, doogie_msgs::MazeCell cell_walls) {
  if (!cell_walls.visited) return;
  if (cell_walls.north_wall && row < maze_map_.getSize()(0)) this->drawNorthWall(row, column);
  if (cell_walls.south_wall && row > 0) this->drawSouthWall(row, column);
  if (cell_walls.east_wall && column < maze_map_.getSize()(1)) this->drawEastWall(row, column);
  if (cell_walls.west_wall && column > 0) this->drawWestWall(row, column);
}

void MazeMap::publishMap() {
  if (is_to_update_map_) {
    this->drawWalls(doogie_pose_.position.row, doogie_pose_.position.column, cell_walls_);
    is_to_update_map_ = false;
  }

  grid_map::GridMapRosConverter::toOccupancyGrid(maze_map_, "ground", 0, 1.0, maze_map_grid_);
  maze_map_pub_.publish(maze_map_grid_);
}

void MazeMap::run() {
  ros::Rate rate(pub_rate_);
  
  while (ros::ok()) {
    this->publishMap();
    ros::spinOnce();
    rate.sleep();
  }
}

void MazeMap::doogiePoseCallback(const doogie_msgs::DoogiePoseConstPtr& doogie_pose) {
  this->doogie_pose_ = *doogie_pose;
}

void MazeMap::mazeObstacleMatrixCallback(const doogie_msgs::MazeCellMultiArrayConstPtr& maze_obstacle_matrix) {
  cell_walls_ = MazeMatrixHandle::getMazeMatrixCell(*maze_obstacle_matrix,
                                                    doogie_pose_.position.row, doogie_pose_.position.column);
  is_to_update_map_ = true;
}

}  // namespace doogie_rviz
