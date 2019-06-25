AUV GNC: Guidance, Navigation, and Control for AUVs
===============================================================
CAUTION: This library is still in development. Use at your own risk.

## Overview
Guidance, Navigation, and Control (GNC) are three of the biggest challenges that all robots face, let alone autonomous robots. From unmanned aerial vehicles (UAVs) to self-driving cars to autonomous underwater vehicles (AUVs), every one of these types of robots must be capable of localizing, navigating, and guiding itself in an unknown environment, and also be capable of controlling its own movements. This library is specifically targeted at AUVs.

Finding an opensource "GNC-like" library for AUVs used to be a struggle, until now. AUVs are becomming increasingly popular, yet there never seemed to exist a solid GNC platform from which anyone can build off of. The purpose of this library is to do just that. AUV GNC is a C++ library containing (almost) all of the needed GNC-related code needed to start working with an AUV.

**Author: [Ted Sender](https://github.com/tsender) (sender.9@osu.edu)**

**Former Affiliation: The Ohio State University's [Underwater Robotics Team](https://uwrt.engineering.osu.edu/), AUVSI [RoboSub](https://www.robonation.org/competition/robosub)**

## Building/Installation
### System Dependencies
AUV GNC is currently targeted towards the Robot Operatin System ([ROS1](https://www.ros.org/)), which must be installed to build the library as is. The library contains the following system dependencies:
* [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html), minimum version 3 (C++ linear algebra library). Note: this already comes pre-installed with ROS, but you may use newer versions if you like
* [Ceres Solver](http://ceres-solver.org/) (C++ non-linear solver developed by Google)
* [CppAD](https://coin-or.github.io/CppAD/doc/cppad.htm) (C++ automatic differentiation library)

### Included System Dependencies
This library has two other dependencies, but these are included in the repo for your convenience:
* [Control Toolbox](https://github.com/ethz-adrl/control-toolbox) (C++ optimized control library developed by researchers at ETH Zurich's Agile & Dexterous Robotics Lab)
* [Kindr](https://github.com/ANYbotics/kindr) (C++ robotics library, only included because the Control Toolbox depends on it)

AUV GNC only relies on the Control Toolbox for optimal control solvers, but because the Control Toolbox depends on Kindr, both libraries are included.

### Building
To get started, first setup a catkin workspace and clone the library into it

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    git clone --recursive https://github.com/tsender/auv_gnc.git
    cd ..

The suggested way to build/install this library is with [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html). Then build the `auv_gnc` package, with the command

    catkin build auv_gnc

The `auv_gnc` package is the main ROS-related packaged inside the library, and it will automatically build its dependent catkin/cmake packages. While you may also build the entire repo with the command

    catkin build
  
it is unnecessary to build all of the Control Toolbox packages since AUV GNC is only dependent on two of them. Building the entire repo actually takes quite a while due to all the overhead in the Control Toolbox, namely due to the `ct_models` package.

### Installing to /opt/ros/kinetic
If you wish to install AUV GNC to the same location as where ROS Kinetic is installed, then you can run these commands (assuming you have already cloned the repo)

    cd ~/catkin_ws/src
    sudo su
    source /opt/ros/kinetic/setup.bash
    catkin build auv_gnc --cmake-args -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic
    exit
