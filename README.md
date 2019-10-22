# doogie_simulator

## Overview

Simulator ROS packages for the Doogie Mouse.

**Keywords:** Micromouse, doogie, Gazebo, ROS

### Simulators Available

- [doogie_gazebo] - Gazebo simulation for Doogie Mouse

### Dependencies 
- [doogie_base] (the stack with all common packages for Doogie Mouse)

### Publications

If you use this work in an academic context, please cite the following publication(s):

**TODO**
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

In order to build **doogie_simulator**, its necessary build its dependencie [doogie_base]. But first, lets create a catkin workspace.

    mkdir -p catkin_ws/src

Now, clone the [doogie_base] repository inside your workspace source.

	cd catkin_workspace/src
	git clone http://github.com/doogie-mouse/doogie_base.git

Then, clone **doogie_simulator** also in your workspace source.
        
    git clone http://github.com/doogie-mouse/doogie_base.git

Now, you can build your catkin workspace.

    cd catkin_ws
    catkin build

## Usage

**Launch Doogie Mouse at Gazebo**:

Just launch the robot.launch

	roslaunch doogie_gazebo robot_launch.launch


[doogie_base]: http://github.com/doogie-mouse/doogie_base.git
[doogie_gazebo]: (/doogie_gazebo) 