# doogie_gazebo

## Overview

This ROS catkin package provides all launchfiles required to simulate Doogie Mouse in Gazebo.

**Keywords:** Micromouse, IEEE, Gazebo, ROS

### Dependencies 
- [gazebo_ros] (Provides all ROS message and service publishers for interfacing with Gazebo through ROS),
- [doogie_description] (package with doogie URDF),
- [doogie_control] (package with doogie ros controllers);

### License

The source code is released under a [GPLv3](/LICENSE).

**Author: [Caio Amaral]<br />
Affiliation: [BIR - Brazilian Institute of Robotics]<br />
Maintainer: Caio Amaral, caioaamaral@gmail.com**

### Supported Versions

The **doogie_gazebo** package has been tested under [ROS] Kinetic and Ubuntu 16.04 using Gazebo 7.0.0 (the one that comes with [ROS] Kinetic Desktop-full installation). 

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) TODO


![Example image](doc/example.jpg) TODO


## Launch files

* **robot_launch.launch:** spawn Doogie Mouse at Gazebo empty.world

  - **Arguments to set Doogie Mouse spawn**

    - **`robot_name:`** specifie a nickname for Doogie Mouse when in gazebo.
      
      - Default: `doogie`
    - **`x:`** set x position coordinate where the robot will be spawned.
      - Default:`0.0`
    - **`y:`** set y position coordinate where the robot will be spawned.
      - Default:`0.0`
    - **`z:`** set z position coordinate where the robot will be spawned.
      - Default: `0.02` --> <span style="color:red">**This will probably change to '0.0' in final version**</span>

  - **Arguments to set Gazebo World**

    - **`paused:`** start Gazebo in a paused state. 

      - Default: `false`.
  
    - **`use_sim_time:`** tells if nodes will use time published at /clock.

      - Default: `true` 

    - **`gui:`** load Gazebo user interface display.
    
      - Default: `true`

    - **`debug:`** start Gazebo Server (gzserver) at debug mode using gdb.

      - Default: `false`

    - **`physics:`** specifie wich physics engine will be used by Gazebo.

      - Default: `ode`

    - **`verbose:`** run gzserver and Gazebo Client (gzclient)in verbose mode (i.e, printing errors and warnings to the terminal).
  
      - Default: `false`

    - **`world_name:`** tells gazebo which world will be loaded.
    
      - Default: `worlds/empty_world` 
    
        **Note:** the **world_name** path is with respect to **GAZEBO_RESOURCE_PATH** environmental variable.

## Nodes

The robot model is controlled by [ros_control / gazebo_ros_control] integration. 

[doogie_control] provides Gazebo simulation the diff_driver_controller to control the robot. 

### ros_package_template

Reads temperature measurements and computed the average.


#### Subscribed Topics

* **`/temperature`** ([sensor_msgs/Temperature])

	The temperature measurements from which the average is computed.


#### Published Topics

...


#### Services

* **`get_average`** ([std_srvs/Trigger])

	Returns information about the current average. For example, you can trigger the computation from the console with

		rosservice call /ros_package_template/get_average


#### Parameters

* **`subscriber_topic`** (string, default: "/temperature")

	The name of the input topic.

* **`cache_size`** (int, default: 200, min: 0, max: 1000)

	The size of the cache.


### NODE_B_NAME

...


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](TODO).

[doogie_description]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_description

[doogie_control]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_control

[doogie_simulators]: https://github.com/Brazilian-Institute-of-Robotics/doogie_simulators

[gazebo_ros]: http://wiki.ros.org/gazebo_ros

[ros_control / gazebo_ros_control]: http://gazebosim.org/tutorials?tut=ros_control

[ROS]: http://www.ros.org

[URDF]: http://wiki.ros.org/urdf 

[Xacro]: http://wiki.ros.org/xacro 

[Rviz]: http://wiki.ros.org/rviz

[BIR - Brazilian Institute of Robotics]: https://github.com/Brazilian-Institute-of-Robotics

[Caio Amaral]: https://github.com/caioaamaral