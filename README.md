Robot-Centric Elevation Mapping
======================

**WARNING: Writing of this document is still under progress!** 

Overview
---------------

This is a collection of [ROS] packages developed for local elevation mapping with a mobile robot. The software is designed for (local) navigation tasks with robots which are equipped with a pose estimation (e.g. IMU & odometry) and a distance sensor (e.g. kinect, laser range sensor). The provided elevation map is limited around the robot and reflects the pose uncertainty that is aggregated through the motion of the robot (robot-centric mapping). This method is developed to explicitly handle drift of the robot pose estimation.

The Robot-Centric Elevation Mapping packages have been tested under ROS Groovy and Ubuntu 13.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author: Peter Fankhauser, pfankhauser@ethz.ch<br />
Affiliation: Autonomous Systems Lab, ETH Zurich**

Citing
---------------

The robot-centric elevation mapping methods used in this software are described in the following paper (available [here]()):

P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart,
**"Robot-centric elevation mapping with uncertainty estimates"**,
in Climbing and Walking Robots (CLAWAR), 2014.

    @inproceedings{Fankhauser2014RobotCentricElevationMapping,
      author = {Fankhauser, Péter and Bloesch, Michael and Gehring, Christian and Hutter, Marco and Siegwart, Roland},
      title = {Robot-centric elevation mapping with uncertainty estimates},
      booktitle = {Climbing and Walking Robots (CLAWAR)},
      year = {2014}
    }

Watch the [video](http://www.youtube.com/watch?v=I9eP8GrMyNQ) demonstrating the Robot-Centric Elevation Mapper: 
[![Robot-Centric Elevation Mapping](example.jpg)](http://www.youtube.com/watch?v=I9eP8GrMyNQ)

Installation
------------

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the Robot-Centric Elevation Mapper depends on following software:

- [Eigen](http://eigen.tuxfamily.org) (linear algebra library),
- [kindr](http://github.com/ethz-asl/kindr) (kinematics and dynamics library for robotics),
- [Schweizer-Messer](http://github.com/ethz-asl/Schweizer-Messer) (programming tools for robotics),
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing).

### Building

In order to install the Robot-Centric Elevation Mapper, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone https://github.com/ethz-asl/elevation_mapping.git
    cd ../
    catkin_make

### Unit Tests

Run the unit tests with

    catkin_make run_tests
    
### Debugging

Build in Debug mode with

    catkin_make -DCMAKE_BUILD_TYPE=Debug
    
You can play a bag file with

    rosparam set use_sim_time true
    rosbag play --clock --rate=1.0 --start=0.0 XXX.bag 

Nodes
------------

### Node: elevation_mapping

#### Services

* **`trigger_fusion`** ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html))

    Trigger the fusing process for the entire elevation map and publish it.
    
    For example, you can trigger the map fusion step from the console with

        rosservice call /elevation_mapping/trigger_fusion
    
* **`get_submap`** (elevation_mapping/GetSubmap)

    Get a fused elevation submap for a requested position and size.
    
    For example, you can get the fused elevation submap at position (-0.5, 0.0) and size (0.5, 1.2) and safe it to a text file form the console with

        rosservice call -- /elevation_mapping/get_submap -0.5 0.0 0.5 1.2 > elevation_submap.txt

### Node: elevation_map_visualization

### Node: elevation_map_msg

Definition of the elevation message and services types. It also contains helper functions that facilitate the handling and conversion of elevation messages.

Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/elevation_mapping/issues).

[ROS]: http://www.ros.org

