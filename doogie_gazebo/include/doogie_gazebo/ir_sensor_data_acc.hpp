#ifndef IR_SENSOR_DATA_ACC_HPP_
#define IR_SENSOR_DATA_ACC_HPP_

#include <vector>
#include <string>

#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Range.h>
#include "doogie_msgs/DoogiePosition.h"

namespace doogie_gazebo {

/**
 * @brief Join the data provided by IR (InfraRed) sensor of the Doogie Mouse simulation 
 * 
 * When IR sensors are added to a robot model in the gazebo, is created a single
 * topic for each sensor. Thus, to simplify the data management of the IR sensor
 * by other nodes, this class was created to publish in a single topic the data of
 * the robot's IR sensors.
 * 
 */
class IRSensorDataAcc {
 public:
  /**
   * @brief Construct a new IRSensorDataAcc object
   * 
   */
  IRSensorDataAcc();
  /**
   * @brief Load parameters, advertise publisher and subscribe in the essential topics. 
   * 
   */
  void init();
  /**
   * @brief Run the application to join the IR sensors data.
   * 
   */
  void run();

 private:
  /**
   * @brief Load parameters from parameter server.
   * 
   */
  void loadParameters();
  /**
   * @brief Callback called by the message filter when all IR sensors data is published.
   * 
   * @param left_ir Data pointer of the left IR sensor.
   * @param front_left_ir Data pointer of the front left IR sensor.
   * @param front_right_ir Data pointer of the front right IR sensor.
   * @param right_ir Data pointer of the right IR sensor.
   */
  void accCallback(const sensor_msgs::RangeConstPtr& left_ir,
                    const sensor_msgs::RangeConstPtr& front_left_ir,
                    const sensor_msgs::RangeConstPtr& front_right_ir,
                    const sensor_msgs::RangeConstPtr& right_ir);
  /**
   * @brief Callback called when is performed a requisition of all IR sensors data.
   * 
   * @param msg Not used in this application.
   */
  void UpdateMatrixCallback(const doogie_msgs::DoogiePositionConstPtr& msg);

  /** Global NodeHandle to publishers and subsribers. */
  ros::NodeHandle nh_;
  /** Private NodeHandle to load parameters. */
  ros::NodeHandle ph_;
  /** Publisher to publish the message with all IR sensors data. */
  ros::Publisher ir_sensors_pub_;
  /** Subscriber used to request all IR sensors data. */
  ros::Subscriber update_matrix_sub_;

  /** Subcriber of left IR sensor topic. */
  message_filters::Subscriber<sensor_msgs::Range> left_ir_sensor_sub_;
  /** Subcriber of front left IR sensor topic. */
  message_filters::Subscriber<sensor_msgs::Range> front_left_ir_sensor_sub_;
  /** Subcriber of front right IR sensor topic. */
  message_filters::Subscriber<sensor_msgs::Range> front_right_ir_sensor_sub_;
  /** Subcriber of right IR sensor topic. */
  message_filters::Subscriber<sensor_msgs::Range> right_ir_sensor_sub_;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Range, sensor_msgs::Range,
                                                          sensor_msgs::Range, sensor_msgs::Range> SyncPolicy;
  /** Object used to sync the data of all IR sensors data. */
  message_filters::Synchronizer<SyncPolicy> ir_sensors_sync_;

  /** Flag used to publish the all IR sensors data asynchronously. */
  bool is_to_publish_wall_distances_;
  /** Store all ir sensors names from paramater server. */
  std::vector<std::string> ir_sensor_names_;
};

}

#endif
