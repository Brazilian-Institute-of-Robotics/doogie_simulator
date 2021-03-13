#ifndef DOOGIE_GAZEBO_GROUND_TRUTH_HPP_
#define DOOGIE_GAZEBO_GROUND_TRUTH_HPP_

#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include "doogie_localization/base_localization.hpp"

namespace doogie_gazebo {

class GroundTruth : public doogie_localization::BaseLocalization {
 public:
  GroundTruth();
  explicit GroundTruth(const std::string& odom_frame);
  virtual ~GroundTruth() = default;
  void odomCallBack(const nav_msgs::OdometryConstPtr &odometry_msg);
  double getCurrentXPosition() override;
  double getCurrentYPosition() override;
  double getCurrentYawOrientation() override;
  double getCurrentNormalizedYawOrientation() override;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pn_{"~"};

  std::string odom_frame_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  tf2_ros::TransformBroadcaster tf_broadcaster_;  
  
  message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
  tf2_ros::MessageFilter<nav_msgs::Odometry> tf2_filter_;

  geometry_msgs::PoseStamped current_pose_;
  geometry_msgs::Twist current_velocity_;
};

}  // doogie_gazebo

#endif // DOOGIE_GAZEBO_GROUND_TRUTH_HPP_