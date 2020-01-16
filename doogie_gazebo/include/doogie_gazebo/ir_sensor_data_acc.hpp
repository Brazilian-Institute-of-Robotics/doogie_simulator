#ifndef IR_SENSOR_DATA_ACC_HPP_
#define IR_SENSOR_DATA_ACC_HPP_

#include <vector>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <sensor_msgs/Range.h>
#include "doogie_msgs/WallDistances.h"

namespace doogie_gazebo {

class IRSensorDataAcc {
 public:
  IRSensorDataAcc();
  void run();

 private:
  void acc_callback(const sensor_msgs::RangeConstPtr& left_ir,
                    const sensor_msgs::RangeConstPtr& front_left_ir,
                    const sensor_msgs::RangeConstPtr& front_right_ir,
                    const sensor_msgs::RangeConstPtr& right_ir);

  ros::NodeHandle nh_;
  std::vector<message_filters::Subscriber<sensor_msgs::Range>> ir_sensors_subs_;
};

}

#endif