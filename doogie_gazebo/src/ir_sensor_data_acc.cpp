#include <sstream>
#include "doogie_gazebo/ir_sensor_data_acc.hpp"
#include "doogie_msgs/WallDistances.h"

namespace doogie_gazebo {

IRSensorDataAcc::IRSensorDataAcc() 
  : ph_("~")
  , ir_sensors_sync_(SyncPolicy(10), left_ir_sensor_sub_, front_left_ir_sensor_sub_,
                     front_right_ir_sensor_sub_, right_ir_sensor_sub_)
  , is_to_publish_wall_distances_(false) {}

void IRSensorDataAcc::loadParameters() {
  if(!ph_.hasParam("ir_sensor/names"))
    throw std::runtime_error("ir_sensor/names not found in parameter server");

  ph_.getParam("ir_sensor/names", ir_sensor_names_);

  if (ir_sensor_names_.size() < 4) {
    std::stringstream error_msg;
    error_msg << "Needed 4 ir sensor names. " << ir_sensor_names_.size() << " was passed.";
    throw std::runtime_error(error_msg.str());
  }
}

void IRSensorDataAcc::init() {
  this->loadParameters();

  std::vector<message_filters::Subscriber<sensor_msgs::Range>*> ir_sensor_subs{&left_ir_sensor_sub_, 
                                                                               &front_left_ir_sensor_sub_,
                                                                               &front_right_ir_sensor_sub_,
                                                                               &right_ir_sensor_sub_};

  for (int i = 0; i < ir_sensor_subs.size(); i++) {
    ir_sensor_subs[i]->subscribe(nh_, "sensor/" + ir_sensor_names_[i], 1);
  }

  ir_sensors_sync_.registerCallback(boost::bind(&IRSensorDataAcc::accCallback, this, _1, _2, _3, _4));
  ir_sensors_pub_ = nh_.advertise<doogie_msgs::WallDistances>("wall_distances", 1);
  update_matrix_sub_ = nh_.subscribe("doogie_pose", 1, &IRSensorDataAcc::UpdateMatrixCallback, this);
}

void IRSensorDataAcc::run() {
  ROS_INFO("IR sensors data accumulator has started!");
  ros::spin();
}

void IRSensorDataAcc::accCallback(const sensor_msgs::RangeConstPtr& left_ir,
                  const sensor_msgs::RangeConstPtr& front_left_ir,
                  const sensor_msgs::RangeConstPtr& front_right_ir,
                  const sensor_msgs::RangeConstPtr& right_ir) {
  if (!is_to_publish_wall_distances_) return;
  
  doogie_msgs::WallDistances wall_distances;

  wall_distances.left_sensor = *left_ir;
  wall_distances.front_left_sensor = *front_left_ir;
  wall_distances.front_right_sensor = *front_right_ir;
  wall_distances.right_sensor = *right_ir;

  ir_sensors_pub_.publish(wall_distances);
  
  is_to_publish_wall_distances_ = false;
}

void IRSensorDataAcc::UpdateMatrixCallback(const doogie_msgs::DoogiePoseConstPtr& msg) {
  is_to_publish_wall_distances_ = true;
}

}
