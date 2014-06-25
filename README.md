Robot-Centric Elevation Mapping
======================

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

    catkin_make run_tests_elevation_map_msg run_tests_elevation_mapping


Basic Usage
------------

In order to get the Robot-Centric Elevation Mapper to run with your robot, you will need to adapt a few parameters. It is the easiest if duplicate and adapt all the parameter files that you need to change in `elevation_mapping/parameters/`. Then, change the entries in the launch-file `elevation_mapping/launch/elevation_mapping.launch` to point at your parameter files. You can then launch the elevation mapping node with

    roslaunch elevation_mapping elevation_mapping.launch

Proceed in the same way for the elevation map visualization by adapting the launch-file `elevation_map_visualization/launch/elevation_map_visualization.launch`. You can then launch the elevation map visualization node with

    roslaunch elevation_map_visualization elevation_map_visualization.launch

Use [rviz] to visualize the elevation map. A sample [rviz] configuration file is provided under `/elevation_map_visualization/rviz/rviz_configuration.rviz`.


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

* **`/elevation_map`** ([elevation_map_msg/ElevationMap])

    The entire (fused) elevation map. Is only published after the `trigger_fusion` service is called.
    
* **`/elevation_map_raw`** ([elevation_map_msg/ElevationMap])

    The entire (raw) elevation map before the fusion step.


#### Services

* **`trigger_fusion`** ([std_srvs/Empty])

    Trigger the fusing process for the entire elevation map and publish it. For example, you can trigger the map fusion step from the console with

        rosservice call /elevation_mapping/trigger_fusion
    
* **`get_submap`** ([elevation_map_msg/GetSubmap])

    Get a fused elevation submap for a requested position and size. For example, you can get the fused elevation submap at position (-0.5, 0.0) and size (0.5, 1.2) and safe it to a text file form the console with

        rosservice call -- /elevation_mapping/get_submap -0.5 0.0 0.5 1.2 false false > elevation_submap.txt


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


### Node: elevation_map_visualization

This node subscribes to an elevation map topic and publishes the corresponding marker array message that can be visualized in [rviz].


#### Subscribed Topics

* **`/elevation_map`** ([elevation_map_msg/ElevationMap])

    The elevation map (fused or raw) to visualize.


#### Published Topics

* **`/elevation_map_marker_array`** ([visualization_msgs/MarkerArray])

    The elevation map marker array that can be visualized in [rviz]. There are multiple visualization namespaces/layers, which show different elements of the elevation map and can be activated separately in [rviz]:
    
    * **`elevation`**: Elevation data of the map displayed as cubes,

    * **`map_region`**: Region of the elevation map as border line,

    * **`variance`**: Variance data of the map shown as sigma bounds with dots (spheres) above the cells.


#### Parameters

*Note: Color values are in RGB form as concatenated integers (for each channel value 0-255). The values can be generated like [this](http://www.wolframalpha.com/input/?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D0%2C+g%3D255%2C+b%3D0%7D) as an example for the color green (red: 0, green: 255, blue: 0).*

* **`elevation_map_topic`** (string, default: "/elevation_mapping/elevation_map")
 
    The name of the elevation map topic to be visualized.

* **`elevation/marker_height`** (double, default: 0.25, min: 0.0)
 
    The height of the cube markers (in m).

* **`elevation/show_empty_cells`** (bool, default: false)
 
    If true, empty cells are also shown in visualization.

* **`elevation/base_color`** (int, default: 1323750 (blue))
 
    The base color for the map visualization (if not overwritten with other color values).

* **`elevation/empty_cell_color`** (int, default: 5263440 (gray))
 
    The color for the empty cells in visualization.

* **`elevation/set_color_from_map`** (bool, default: true)
 
    If true, colors for the elevation visualization are based on the map color data.

* **`elevation/set_color_from_variance`** (bool, default: false)
 
    If true, colors for the elevation visualization are based on the map variance data.

* **`elevation/set_color_from_height`** (bool, default: false)
 
    If true, colors for the elevation visualization are based on the map elevation data.

* **`elevation/set_saturation_from_variance`** (bool, default: true)
 
    If true, the color saturations for the elevation visualization reflect the variance data.

* **`elevation/set_alpha_from_variance`** (bool, default: false)
 
    If true, the marker transparency for the elevation visualization reflect the variance data.

* **`elevation/lower_variance_color`**, **`elevation/upper_variance_color`** (int, default: 255 (blue), 16711680 (red))
 
    The colors for the lower and upper variance values (if `elevation/set_color_from_variance` is set to `true`).

* **`elevation/variance_lower_value`**, **`elevation/variance_upper_value`** (double, default: 0.0, pow(0.03, 2))
 
    The lower and upper value of the variance for mapping from the variance data to color properties.

* **`elevation/elevation_lower_value`**, **`elevation/elevation_upper_value`** (double, default: -0.5, 0.5)
 
    The lower and upper value of the elevation for mapping from the elevation data to color properties.

* **`elevation/min_marker_saturation`**, **`elevation/max_marker_saturation`** (double, default: 0.0, 1.0)
 
    The minimum and maximum saturation of the marker colors.

* **`elevation/min_marker_alpha`**, **`elevation/max_marker_alpha`** (double, default: 0.2, 1.0)
 
    The minimum and maximum transparency of the markers.

* **`map_region/color`** (int, default: 16777215 (white))
 
    The color of the map region visualization.

* **`map_region/line_width`** (double, default: 0.003)
 
    The line width of the map region marker (in m).

* **`variance/sigma_bound`** (double, default: 1.0)

    The Sigma bound that is visualized.

* **`variance/marker_size`** (double, default: 0.003)

    The size of the markers (diameter of the spheres in m).

* **`variance/base_color`** (int, default: 16711680 (red))

    The base color for the variance visualization (if not overwritten with other color values).

* **`variance/base_color`** (int, default: 16711680 (red))

    The base color for the variance visualization (if not overwritten with other color values).

* **`variance/set_color_from_variance`** (bool, default: true)
 
    If true, colors for the variance visualization are based on the map variance data.

* **`variance/lower_variance_color`**, **`variance/upper_variance_color`** (int, default: 255 (blue), 16711680 (red))
 
    The colors for the lower and upper variance values (if `variance/set_color_from_variance` is set to `true`).

* **`variance/variance_lower_value`**, **`variance/variance_upper_value`** (double, default: 0.0, pow(0.03, 2))
 
    The lower and upper value of the variance for mapping from the variance data to color properties.


### Package: elevation_map_msg

Definition of the elevation map message type and services. It also contains helper functions that facilitate the handling and conversion of elevation map messages.

* **`ElevationMap`** ([elevation_map_msg/ElevationMap])

    Definition of the elevation map message type.

* **`GetSubmap`** ([elevation_map_msg/GetSubmap])

    Definition of the service for requesting an elevation submap.


Bugs & Feature Requests
------------

Please report bugs and request features using the [Issue Tracker](https://github.com/ethz-asl/elevation_mapping/issues).


[ROS]: http://www.ros.org
[rviz]: http://wiki.ros.org/rviz
[elevation_map_msg/ElevationMap]: elevation_map_msg/msg/ElevationMap.msg
[sensor_msgs/PointCloud2]: http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html
[geometry_msgs/PoseWithCovarianceStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html
[std_srvs/Empty]: http://docs.ros.org/api/std_srvs/html/srv/Empty.html
[elevation_map_msg/GetSubmap]: elevation_map_msg/srv/GetSubmap.srv
[visualization_msgs/MarkerArray]: http://docs.ros.org/api/visualization_msgs/html/msg/MarkerArray.html
[geometry_msgs/PolygonStamped]: http://docs.ros.org/api/geometry_msgs/html/msg/PolygonStamped.html

