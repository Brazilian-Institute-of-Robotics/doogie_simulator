# **doogie_simulator**

The **`doogie_simulator`** stack provides a [ROS] interface communication to simulate Doogie Mouse plataform. For tutorials, check [doogie_gazebo/Tutorials] on wiki.

**Keywords:** Micromouse, Gazebo, ROS

**Author: [Caio Amaral]<br />
Affiliation: [BIR - Brazilian Institute of Robotics]<br />
Maintainer: Caio Amaral, caioaamaral@gmail.com**

## Supported Versions

- **Kinetic**: Built and tested under [ROS] Kinetic and Ubuntu 16.04

<!-- [![Build Status](http://rsl-ci.ethz.ch/buildStatus/icon?job=ros_best_practices)](http://rsl-ci.ethz.ch/job/ros_best_practices/) TODO -->

## Dependencies 
- [doogie_base] : ROS packages stack with all common packages for working with Doogie Mouse

# **Table of Contents**
- [**doogie_simulator**](#doogiesimulator)
  - [Supported Versions](#supported-versions)
  - [Dependencies](#dependencies)
- [**Table of Contents**](#table-of-contents)
- [**Simulators Supported**](#simulators-supported)
  - [Gazebo](#gazebo)
    - [Publications](#publications)
- [**Installation**](#installation)
  - [Installation from Packages](#installation-from-packages)
  - [Building from Source](#building-from-source)
  - [Example of Usage](#example-of-usage)
    - [Launch Doogie Mouse at **Gazebo**:](#launch-doogie-mouse-at-gazebo)
- [**Purpose of the Project**](#purpose-of-the-project)
- [License](#license)
- [**Bugs & Feature Requests**](#bugs--feature-requests)

# **Simulators Supported**

## Gazebo

- [doogie_gazebo] : ROS package with all launchfiles required to simulate Doogie Mouse in Gazebo

### Publications

If you use this work in an academic context, please cite the following publication(s):

* C. Amaral, M. Santos Meneses: **DOOGIE MOUSE: UMA PLATAFORMA OPEN SOURCE PARA
APLICAÇÃO DE ALGORITMOS INICIAIS DE INTELIGÊNCIA
ARTIFICIAL EM ROBÓTICA MÓVEL**. V SIINTEC International Symposium on Innovation and Technology, 2019. ([PDF](/doogie_gazebo/docs/Paper_SIINTEC_V.pdf))

        @inproceedings{Doogie2019,
            author = {Amaral, C. and Santos Meneses, M. },
            booktitle = {V SIINTEC International Symposium on Innovation and Technology},
            title = {{DOOGIE MOUSE: UMA PLATAFORMA OPEN SOURCE PARA APLICAÇÃO DE ALGORITMOS INICIAIS DE INTELIGÊNCIA ARTIFICIAL EM ROBÓTICA MÓVEL}},
            publisher = {CIMATEC},
            year = {2019}
        }

____

# **Installation**

## Installation from Packages

TODO

    sudo apt-get install ros-indigo-...

</br>

## Building from Source

If you haven't installed [ROS] yet, please check [Ubuntu install of ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu). Desktop-Full Install is the recommended one in order to work with this repository.    

**Building:**

First, lets create a catkin workspace.

    mkdir -p ~/catkin_ws/src

**doogie_simulator** depends on [doogie_base] stack. So let's clone it inside our workspace source and install it.

	cd ~/catkin_workspace/src
	git clone http://github.com/doogie-mouse/doogie_base.git
    rosdep install --from-paths src -ignore-src -r -y

Then, clone **doogie_simulator** also in your workspace source.
        
    git clone http://github.com/doogie-mouse/doogie_simulator.git

Now, just build your catkin workspace.

    cd ~/catkin_ws
    catkin build

Don't forget to source your workspace before using it.
    
    source devel/setup.bash
</br>

## Example of Usage

### Launch Doogie Mouse at **Gazebo**:

Just launch the robot.launch

	roslaunch doogie_gazebo robot_launch.launch

</br>

# **Purpose of the Project**
</br>

# License

Doogie Mouse Simulator source code is released under a [GPLv3](/LICENSE).

</br>

# **Bugs & Feature Requests**

Please report bugs and request features using the [Issue Tracker].


[doogie_base]: http://github.com/doogie-mouse/doogie_base.git
[doogie_gazebo]: doogie_gazebo
[doogie_gazebo/Tutorials]: http://github.com/doogie-mouse/doogie_simulator/wiki/doogie_gazebo
[Issue Tracker]: http://github.com/doogie-mouse/doogie_simulator/issues
[ROS]: http://www.ros.org