#include "doogie_gazebo/ground_truth.hpp"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace doogie_gazebo {

GroundTruth::GroundTruth() : GroundTruth("odom") {}

GroundTruth::GroundTruth(const std::string& odom_frame)
: odom_frame_(odom_frame)
, tf2_filter_(odom_sub_, tf_buffer_, odom_frame, 1, nullptr) {
  std::string topic_name = "/gazebo/ground_truth";
  pn_.getParam("input_topic", topic_name);
  odom_sub_.subscribe(nh_, topic_name, 1);
  tf2_filter_.registerCallback(boost::bind(&GroundTruth::odomCallBack, this, _1));
}

void GroundTruth::odomCallBack(const nav_msgs::OdometryConstPtr &odometry_msg) {
  ROS_DEBUG_THROTTLE(0.5, "odom callback");
  current_pose_.header = odometry_msg->header;
  current_pose_.pose = odometry_msg->pose.pose;
  current_velocity_ = odometry_msg->twist.twist;

  tf_buffer_.transform(current_pose_, current_pose_, odom_frame_);
  ROS_INFO_THROTTLE(1.0, "Doogie Position(x:%f y:%f z:%f)\n", 
            current_pose_.pose.position.x,
            current_pose_.pose.position.y,
            current_pose_.pose.position.z);
  
  tf2::Stamped<tf2::Transform> temp;
  tf2::fromMsg(current_pose_, temp);
  geometry_msgs::TransformStamped gt_tf = tf2::toMsg(temp);
  gt_tf.child_frame_id = "ground_truth";
  tf_broadcaster_.sendTransform(gt_tf);
}

double GroundTruth::getCurrentXPosition() {
  return current_pose_.pose.position.x;
}

double GroundTruth::getCurrentYPosition() {
  return current_pose_.pose.position.y;
}

double GroundTruth::getCurrentYawOrientation() {
  return tf2::getYaw(current_pose_.pose.orientation);
}

double GroundTruth::getCurrentNormalizedYawOrientation() {
  return angles::normalize_angle(getCurrentYawOrientation());
}
  
}  // doogie_gazebo

PLUGINLIB_EXPORT_CLASS(doogie_gazebo::GroundTruth, doogie_localization::BaseLocalization)