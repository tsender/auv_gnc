AUV GNC: Guidance, Navigation, and Control for AUVs
===============================================================
This is an open-source, C++ library for guidance, navigation, and control of AUVs.

## Overview
As a former member of The Ohio State University's [Underwater Robotics Team](https://uwrt.engineering.osu.edu/) competing in AUVSI Robonation's [RoboSub](https://www.robonation.org/competition/robosub) competition, I know how difficult it can be for a team to build a strong foundation to perform well in this competition. As engineers, we learn from each other, using other people's works as a starting point for us to build upon. With the advent of the [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator) research project, simulation for AUVs using ROS and Gazebo has become much easier. However, there still did not exist an open-source library dedicated to guidance, navigation, and control of AUVs. I hope this library helps to bridge that gap, making it easier for anyone interested in this field to experiment and test AUVs. AUV GNC is a C++ library containing a fair amount of GNC-related code to help you start working with an AUV. My hope is that you find this library useful, whether you use it as is or improve upon it for your own needs. If you do find it useful, I only ask that in return you give me credit for having created this repository (I do not have any required citation guidelines).

**Author: [Ted Sender](https://github.com/tsender) (tsender@umich.edu)**

## Repo Status
I am currently a PhD student at the University of Michigan and this github project has nothing to do with my research thesis, this was just a side-project I did for fun while I was on the Underwater Robotics Team at OSU. I do not anticipate having time in the near future to update this repo or fix any known problems (see bottom of README). Though, I am happy to provide some guidance, time-permitting, to anyone that wishes to learn more or wants to improve this repo.

## AUV GNC Packages
This library is built on ROS and so all packages are catkin packages. This library also adopts the North East Down (NED) convention throughout.

### auv_core
- Contains essential headers for all other packages in this library including math for 3-D rotations using quaternions and structs specifically for describing the AUV in the `auv_control` package.

### auv_guidance
- Contains the building blocks for creating various trajectories in the water. 
- The Simultaneous Trajectory creates a minimum jerk trajectory from one pose to another in the specified duration. This is the primary building blocks for trajectory generation used in this package.
- The Long Trajectory is used if the vehicle is travelling over relatively large distances, and determines appropriate waypoints along the way for the vehicle to speed up, cruise, and slow down.
- The Basic Trajectory can take the vehicle from one arbitrary pose to another, and is what the user interacts with through ROS. Based on user-specified constraints, this trajectory automatically calculates an appropriate duration of travel for the entire trajectory, determines if it's safe to perform the entire action simultaneously or if it requires a long trajectory, and will calculate the state and acceleration vectors used for the controller. NOTE: At the time of writing this library I did not know how more advanced controllers, such as MPC, work and so I did my best to accomodate any constraints set on vehicle motion.

### auv_navigation
- Contains code for implementing a Kalman Filter and Extended Kalman Filter.
- The primary feature is the translational EKF to estimate the vehicle's state under translational motion.

### auv_control
- Contains code for a continuous-time Linear Quadratic Regulator (LQR) controller.
- This library uses Google's Ceres solver to calculate the nominal thruster forces and uses LQR to account for perturbations between the reference trajectory and the current state.
   - WARNING: The integral action has not been tested yet. Please do not use this feature unless you wish to debug the code.
- At most eight thrusters are supported in this controller.

### auv_gnc
- This is the ROS package that you will be interacting with. 
- Contains the ROS node for the translational EKF, which supports asynchronous data from all AUV sensors (depth, IMU, and DVL). EKF weights are specified in a YAML file.
- Contains the ROS node for the Guidance Controller, which combines both trajectory generation and the LQR controller in a single node. AUV configuration is specified in a YAML file.

### auv_msgs
- Contains custom ROS messages and services for `auc_gnc`.

## Building/Installation
Note: This library has been tested on Ubuntu 16.04 with ROS Kinetic Kame.

### Dependencies
AUV GNC is currently targeted towards the Robot Operating System ([ROS](https://www.ros.org/)), which must be installed to build the library as is. The system dependencies are listed below. It is recommended that you install these via the install script provided (see Building).
* [Eigen](https://eigen.tuxfamily.org/dox/GettingStarted.html)  (C++ linear algebra library).
    * Minimum version 3.3, but repo has only been tested with Eigen 3.3.7
* [Ceres Solver](http://ceres-solver.org/) (C++ non-linear solver developed by Google)
    * Ceres is installed with its own version of Eigen, so it must be installed first
* [CppAD](https://coin-or.github.io/CppAD/doc/cppad.htm) (C++ automatic differentiation library)

### Included Libraries
AUV GNC has two other dependencies listed below, but these libraries are included in the repo for your convenience. AUV GNC only relies on the Control Toolbox for optimal control solvers, however, because the Control Toolbox depends on Kindr, both libraries are included.
* [Control Toolbox](https://github.com/ethz-adrl/control-toolbox) (C++ optimized control library developed by researchers at ETH Zurich's Agile & Dexterous Robotics Lab)
* [Kindr](https://github.com/ANYbotics/kindr) (C++ robotics library, only included because the Control Toolbox depends on it)

### Building
To get started, clone this library into a catkin workspace.

    cd ~/catkin_ws/src
    git clone --recursive https://github.com/tsender/auv_gnc.git

For FIRST TIME users, please run the following to install the required dependencies:

    cd ~/catkin_ws/src/auv_gnc/
    ./install_deps.sh
   
The recommended way to build/install this library is with [Catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html). It is preferred that catkin be configured with the install space as follows

    cd ~/catkin_ws/
    catkin init
    catkin config --install

After initializing the install space, you should see a line indicating that the install space is merged. FInally, build the `auv_gnc` package (this will automatically build all of its dependent catkin/cmake packages).

    catkin build auv_gnc

### Installing to /opt/ros/kinetic
If you wish to install AUV GNC to the same location as where ROS Kinetic is installed to minimize the number of setup.bash files that need to be sourced, then you can run these commands. Before doing so, you must have your catkin workspace configured to use the install space as detailed above. Note, it is prefferred to build the repo normally and simply install it in root.

    cd ~/catkin_ws/
    catkin build auv_gnc
    sudo su
    source /opt/ros/kinetic/setup.bash
    catkin build auv_gnc --cmake-args -DCMAKE_INSTALL_PREFIX=/opt/ros/kinetic
    exit

### Notes
While you may also build the entire repo with the command `catkin build`, it is unnecessary to build all of the Control Toolbox packages since `auv_gnc` is only dependent on `ct_core` and `ct_optcon`. 

Building the entire AUV GNC repo can also take a while due to the additional overhead in the Control Toolbox. Further, if you see an error message saying there is "no target to make ct_doc", do not fret. This is just a documentation package from the Control Toolbox and they are working on a fix.

## How to use AUV GNC
You will need to locate the appropriate YAML files mentioned below in the [auv_gnc/cfg](https://github.com/tsender/auv_gnc/tree/master/auv_gnc/cfg) folder.
You will also need to locate the appropriate launch files mentioned below in the [auv_gnc/launch](https://github.com/tsender/auv_gnc/tree/master/auv_gnc/launch) folder.

### How to launch the Trans EKF node (translational EKF)
See  the `trans_ekf.yaml` as an example file for the configurable parameters. You will also need to create a ROS node that publishes the appropriate data using the SixDoF message in `auv_msgs` (each field is described in the .msg file). If you are all set, then launch your node that publishes the SixDoF message and launch the trans_ekf node:

      roslaunch auv_gnc trans_ekf
   
It is recommended that you place everything into a single launch file in one of your own ROS packages and pass the argument with the YAML file's path. See the `trans_ekf.launch` file for more details.

### How to launch the Guidance Controller node
See  the `gc_config.yal` and `auv_config.yaml` as example files for the configurable parameters. Please read through these YAML files carefully. Please keep the `enable_integrator` field as `false` as this feature is untested. The Guidance Controller uses an action server to receive goal trajectories. Please see the `auv_msgs` package for the service file and its internal messages. When ready, do:

      roslaunch auv_gnc guidance_controller
   
It is recommended that you place everything into a single launch file in one of your own ROS packages and pass the argument with both YAML file paths. See the `guidance_controller.launch` file fore more details.

## Known Problems
Occasionally a set of matrices that is fed into the LQR solver from the Control Toolbox results in an infeasible solution. When this happens, the LQR solver terminates the entire program. I have not gotten around to investigating the exact culrpit of this phenomena, but I believe the reason is that some orientations of the vehicle result in an (A,B) matrix pair that is uncontrollable. A thorough investigation would require determining what orientations cause the error and verifying the uncontrollability of those (A,B) matrices. If anyone wishes to dive into solving this issue, I'm sure anyone that uses this library would appreciate it.

A quick workaround would be to place the `lqrSolver_.compute(Q_, R_, A_, B_, K_);` [line of code](https://github.com/tsender/auv_gnc/blob/master/auv_control/src/auv_lqr.cpp#L272) in a try-catch block for safety (something that deserves to be in the code, I just never got around to doing that).
