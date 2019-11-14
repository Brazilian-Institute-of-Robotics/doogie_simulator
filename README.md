# **doogie_simulator**

The **doogie_simulator** stack provides a [ROS] interface communication to simulate Doogie Mouse plataform. For tutorials, please check [doogie_gazebo/Tutorials] on wiki.

**Keywords:** Micromouse, Gazebo, ROS

**Author: [Caio Amaral]<br />
Affiliation: [BIR - Brazilian Institute of Robotics]<br />
Maintainer: Caio Amaral, caioaamaral@gmail.com**

### Supported Versions

- **Kinetic**: Built and tested under [ROS] Kinetic and Ubuntu 16.04

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) TODO -->

### Dependencies 
- [doogie_base] : ROS packages stack with all common packages for working with Doogie Mouse

</br>

# **Table of Contents**
- [**doogie_simulator**](#doogiesimulator)
    - [Supported Versions](#supported-versions)
    - [Dependencies](#dependencies)
- [**Table of Contents**](#table-of-contents)
- [**Simulators Supported**](#simulators-supported)
    - [Gazebo](#gazebo)
- [**Installation**](#installation)
    - [Building from Source:](#building-from-source)
  - [Example of Usage](#example-of-usage)
    - [Launch Doogie Mouse at Gazebo:](#launch-doogie-mouse-at-gazebo)
- [**Purpose of the Project**](#purpose-of-the-project)
- [**License**](#license)
- [**Bugs & Feature Requests**](#bugs--feature-requests)

# **Simulators Supported**

### Gazebo

- [doogie_gazebo] : ROS package with all launchfiles required to simulate Doogie Mouse in Gazebo

</br>

# **Installation**

<!-- ### 1. Installation from Packages:

TODO

    sudo apt-get install ros-indigo-...


or you could also build this repository from source. -->

### Building from Source:

Attention, if you haven't installed [ROS] yet, please check [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Desktop-Full Install is the recommended one in order to work with this repository.    

**Building:**

First, lets create a catkin workspace.

    mkdir -p ~/doogie_ws/src

**doogie_simulator** depends on [doogie_base] stack. So let's clone it inside our workspace source and install it.

	cd ~/doogie_ws/src
	git clone http://github.com/doogie-mouse/doogie_base.git
    cd ..
    rosdep install --from-paths src --ignore-src -r -y

Then, clone **doogie_simulator** also in your workspace source.
        
    git clone http://github.com/doogie-mouse/doogie_simulator.git

Now, just build your catkin workspace.

    cd ~/doogie_ws
    catkin_make

Don't forget to source your workspace before using it.
    
    source devel/setup.bash


## Example of Usage

### Launch Doogie Mouse at Gazebo:

Just launch the robot.launch

	roslaunch doogie_gazebo robot_launch.launch

</br>

# **Purpose of the Project**

Doogie Mouse platform was originally developed in 2019 as an undergraduate thesis (Theoprax methodoly) at Centro Universitário SENAI CIMATEC in partnership with Brazillian Institute of Robotics, for teaching principles of artificial intelligence using high level framework for writing and reusing robot software.

It's a [open source project](/LICENSE) and expects modifications and contributions from it's users. 

</br>

# **License**

Doogie Mouse Simulator source code is released under a [Apache 2.0](/LICENSE).

</br>

# **Bugs & Feature Requests**

Please report bugs and request features using the [Issue Tracker].

[BIR - Brazilian Institute of Robotics]: https://github.com/Brazilian-Institute-of-Robotics
[Caio Amaral]: https://github.com/caioaamaral
[doogie_base]: http://github.com/doogie-mouse/doogie_base.git
[doogie_gazebo]: doogie_gazebo
[doogie_gazebo/Tutorials]: http://github.com/doogie-mouse/doogie_simulator/wiki/doogie_gazebo
[Issue Tracker]: http://github.com/doogie-mouse/doogie_simulator/issues
[ROS]: http://www.ros.org