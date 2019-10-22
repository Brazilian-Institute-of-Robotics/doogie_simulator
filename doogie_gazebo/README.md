# doogie_gazebo

## Overview

This is a ROS catkin package that provides launchfiles to simulate Doogie Mouse in gazebo.

**Keywords:** Micromouse, IEEE, Gazebo, ROS

### Dependencies 
- [ROS] (middleware for robotics),
- [doogie_description] (package with doogie URDF),
- [doogie_control] (package with doogie ros controllers);

### License

The source code is released under a [BSD 3-Clause license](ros_package_template/LICENSE).

**Author: [Caio Amaral]<br />
Affiliation: [BIR - Brazilian Institute of Robotics]<br />
Maintainer: Caio Amaral, caioaamaral@gmail.com**

The **doogie_gazebo** package has been tested under [ROS] Kinetic and Ubuntu 16.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

[![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) TODO


![Example image](doc/example.jpg) TODO


### Publications

If you use this work in an academic context, please cite the following publication(s):

TODO
* P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart: **PAPER TITLE**. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2015. ([PDF](http://dx.doi.org/10.3929/ethz-a-010173654))

        @inproceedings{Fankhauser2015,
            author = {Fankhauser, P\'{e}ter and Hutter, Marco},
            booktitle = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
            title = {{PAPER TITLE}},
            publisher = {IEEE},
            year = {2015}
        }


## Installation

### Installation from Packages

TODO

    sudo apt-get install ros-indigo-...

### Building from Source

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

	cd catkin_workspace/src
	git clone https://github.com/Brazilian-Institute-of-Robotics/doogie_gazebo.git
	cd ../
	catkin build

## Usage

**Launch Doogie Mouse at Gazebo**:

Just launch the robot.launch

	roslaunch doogie_gazebo robot_launch.launch

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

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/ros_best_practices/issues).


[ROS]: http://www.ros.org

[URDF]: http://wiki.ros.org/urdf 

[Xacro]: http://wiki.ros.org/xacro 

[Rviz]: http://wiki.ros.org/rviz

[doogie_description]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_description

[doogie_control]:  https://github.com/Brazilian-Institute-of-Robotics/doogie_control

[doogie_simulators]: https://github.com/Brazilian-Institute-of-Robotics/doogie_simulators

[BIR - Brazilian Institute of Robotics]: https://github.com/Brazilian-Institute-of-Robotics

[Caio Amaral]: https://github.com/caioaamaral