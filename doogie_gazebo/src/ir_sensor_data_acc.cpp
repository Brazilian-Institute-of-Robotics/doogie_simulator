#include <string>
#include "doogie_gazebo/ir_sensor_data_acc.hpp"

using namespace std;

namespace doogie_gazebo {

IRSensorDataAcc::IRSensorDataAcc() {
  vector<string> topic_names{"sensor/left", "sensor/front_left", "sensor/front_right", "sensor/right"};

  for(string topic_name : topic_names) 
}

void IRSensorDataAcc::run() {

}

}