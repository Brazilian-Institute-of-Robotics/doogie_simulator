#include <ros/ros.h>
#include "doogie_gazebo/ir_sensor_data_acc.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ir_sensor_data_acc_node");

  doogie_gazebo::IRSensorDataAcc ir_sensor_data_acc;

  try {
    ir_sensor_data_acc.init();
    ir_sensor_data_acc.run();
  } catch (std::runtime_error &error) {
    ROS_FATAL("%s", error.what());
    ros::shutdown();
  }

  return 0;
}
