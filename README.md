Robot-Centric Elevation Mapping
======================

Overview
---------------

This is a [ROS] package developed for elevation mapping with a mobile robot. The software is designed for (local) navigation tasks with robots which are equipped with a pose estimation (e.g. IMU & odometry) and a distance sensor (e.g. kinect, laser range sensor, stereo camera). The provided elevation map is limited around the robot and reflects the pose uncertainty that is aggregated through the motion of the robot (robot-centric mapping). This method is developed to explicitly handle drift of the robot pose estimation.

The Robot-Centric Elevation Mapping packages have been tested under ROS Indigo and Ubuntu 14.04. This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

**Author: Péter Fankhauser, pfankhauser@ethz.ch<br />
Affiliation: Autonomous Systems Lab, ETH Zurich**


Citing
---------------

The robot-centric elevation mapping methods used in this software are described in the following paper (available [here](http://dx.doi.org/10.3929/ethz-a-010173654)):

P. Fankhauser, M. Bloesch, C. Gehring, M. Hutter, and R. Siegwart,
**"Robot-Centric Elevation Mapping with Uncertainty Estimates"**,
in International Conference on Climbing and Walking Robots (CLAWAR), 2014.

    @inproceedings{Fankhauser2014RobotCentricElevationMapping,
      author = {Fankhauser, Péter and Bloesch, Michael and Gehring, Christian and Hutter, Marco and Siegwart, Roland},
      title = {Robot-Centric Elevation Mapping with Uncertainty Estimates},
      booktitle = {International Conference on Climbing and Walking Robots (CLAWAR)},
      year = {2014}
    }

Watch the [video](http://www.youtube.com/watch?v=I9eP8GrMyNQ) demonstrating the Robot-Centric Elevation Mapper: 
[![Robot-Centric Elevation Mapping](example.jpg)](http://www.youtube.com/watch?v=I9eP8GrMyNQ)


Installation
------------

### Dependencies

This software is built on the Robotic Operating System ([ROS]), which needs to be [installed](http://wiki.ros.org) first. Additionaly, the Robot-Centric Elevation Mapping depends on following software:

- [Grid Map](https://github.com/ethz-asl/grid_map) (grid map library for mobile robots)
- [kindr](http://github.com/ethz-asl/kindr) (kinematics and dynamics library for robotics),
- [Schweizer-Messer](http://github.com/ethz-asl/Schweizer-Messer) (programming tools for robotics),
- [Point Cloud Library (PCL)](http://pointclouds.org/) (point cloud processing),
- [Eigen](http://eigen.tuxfamily.org) (linear algebra library).


### Building

In order to install the Robot-Centric Elevation Mapping, clone the latest version from this repository into your catkin workspace and compile the package using ROS.

    cd catkin_workspace/src
    git clone https://github.com/ethz-asl/elevation_mapping.git
    cd ../
    catkin_make


### Unit Tests

Run the unit tests with

    catkin_make run_tests_elevation_map_msg run_tests_elevation_mapping


Basic Usage
------------

In order to get the Robot-Centric Elevation Mapping to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files that you need to change in `elevation_mapping/parameters/`. Then, change the entries in the launch-file `elevation_mapping/launch/elevation_mapping.launch` to point at your parameter files. You can then launch the elevation mapping node with

    roslaunch elevation_mapping elevation_mapping.launch

Proceed in the same way for the elevation map visualization by adapting the launch-file `elevation_map_visualization/launch/elevation_map_visualization.launch`. You can then launch the elevation map visualization node with

    roslaunch elevation_mapping visualization.launch

or

    roslaunch elevation_mapping raw_visualization.launch

Use [rviz] to visualize the elevation map. A sample [rviz] configuration file is provided under `/elevation_map_visualization/rviz/elevation_map_visualization.rviz`.


Nodes
------------

### Node: elevation_mapping

This is the main Robot-Centric Elevation Mapping node. It uses the distance sensor measurements and the pose and covariance of the robot to generate an elevation map with variance estimates.


#### Subscribed Topics

* **`/points`** ([sensor_msgs/PointCloud2])

    The distance measurements.
    
* **`/robot_state/pose`** ([geometry_msgs/PoseWithCovarianceStamped])

    The robot pose and covariance.


#### Published Topics

* **`elevation_map`** ([grid_map_msg/GridMap])

    The entire (fused) elevation map. Is only published after the `trigger_fusion` service is called.
    
* **`elevation_map_raw`** ([grid_map_msg/GridMap])

    The entire (raw) elevation map before the fusion step.


#### Services

* **`trigger_fusion`** ([std_srvs/Empty])

    Trigger the fusing process for the entire elevation map and publish it. For example, you can trigger the map fusion step from the console with

        rosservice call /elevation_mapping/trigger_fusion
    
* **`get_submap`** ([grid_map_msg/GetGridMap])

    Get a fused elevation submap for a requested position and size. For example, you can get the fused elevation submap at position (-0.5, 0.0) and size (0.5, 1.2) and safe it to a text file form the console with

        rosservice call -- /elevation_mapping/get_grid_map -0.5 0.0 0.5 1.2 []

* **`clear_map`** ([std_srvs/Empty])

    Initiates clearing of the entire map for resetting purposes. Trigger the map clearing with

        rosservice call /elevation_mapping/clear_map


#### Parameters

* **`point_cloud_topic`** (string, default: "/points")
 
    The name of the distance measurements topic.

* **`robot_pose_topic`** (string, default: "/robot_state/pose")
 
    The name of the robot pose and covariance topic.

* **`base_frame_id`** (string, default: "/robot")
 
    The id of the robot base tf frame.
    
* **`parent_frame_id`** (string, default: "/map")
 
    The id of the parent tf frame of the elevation map.

* **`elevation_map_frame_id`** (string, default: "/elevation_map")
 
    The id of the tf frame of the elevation map.

* **`track_point_frame_id`** (string, default: "/robot")
 
    The elevation map is moved along with the robot following a *track point*. This is the id of the tf frame in which the track point is defined.

* **`track_point_x`**, **`track_point_y`**, **`track_point_z`** (double, default: 0.0, 0.0, 0.0)

    The elevation map is moved along with the robot following a *track point*. This is the position of the track point in the `track_point_frame_id`.
    
* **`robot_pose_cache_size`** (int, default: 200, min: 0)
 
    The size of the robot pose cache.

* **`min_update_rate`** (double, default: 2.0)
 
    The mininum update rate (in Hz) at which the elevation map is updated either from new measurements or the robot pose estimates.

* **`relocate_rate`** (double, default: 3.0)
 
    The rate (in Hz) at which the elevation map is checked for relocation following the tracking point.

* **`length_in_x`**, **`length_in_y`** (double, default: 1.5, min: 0.0)
 
    The size (in m) of the elevation map.

* **`position_x`**, **`position_y`** (double, default: 0.0)
 
    The position of the elevation map (center) in the elevation map frame.

* **`resolution`** (double, default: 0.01, min: 0.0)
 
    The resolution (cell size in m/cell) of the elevation map.

* **`min_variance`**, **`max_variance`** (double, default: 9.0e-6, 0.01)

    The minimum and maximum values for the elevation map variance data.
    
* **`mahalanobis_distance_threshold`** (double, default: 2.5)

    The threshold for the Mahalanobis distance. Decides if measurements are fused with the existing data, overwritten or ignored.
    
* **`multi_height_noise`** (double, default: 9.0e-7)

    Added noise for cell with multiple height measurements (e.g. walls).
    
* **`min_horizontal_variance`**, **`max_horizontal_variance`** (double, default: pow(resolution / 2.0, 2), 0.5)

    The minimum and maximum values for the elevation map horizontal variance data.
    
* **`sensor_cutoff_min_depth`**, **`sensor_cutoff_max_depth`** (double, default: 0.2, 2.0)

    The minimum and maximum values for the length of the distance sensor measurements. Measurements outside this interval are ignored.
    
* **`sensor_model_normal_factor_a`**, **`sensor_model_normal_factor_b`**, **`sensor_model_normal_factor_c`**, **`sensor_model_lateral_factor`** (double)

    The data for the sensor noise model. 


Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/elevation_mapping/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[grid_map_msg/GridMap]: https://github.com/ethz-asl/grid_map/blob/master/grid_map_msg/msg/GridMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[grid_map_msg/GetGridMap]: https://github.com/ethz-asl/grid_map/blob/master/grid_map_msg/srv/GetGridMap.srv
