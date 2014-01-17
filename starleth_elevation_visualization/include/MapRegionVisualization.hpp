/*
 * MapRegionVisualization.hpp
 *
 *  Created on: Jan 17, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// StarlETH Navigation
#include <starleth_elevation_msg/ElevationMap.h>

// ROS
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>

namespace starleth_elevation_visualization {

/*
 *
 */
class MapRegionVisualization
{
 public:
  MapRegionVisualization(ros::NodeHandle& nodeHandle);
  virtual ~MapRegionVisualization();

  bool initialize();
  bool update(const starleth_elevation_msg::ElevationMap& map);
  bool publish();

 private:

  unsigned int getNPoints();

  ros::NodeHandle& nodeHandle_;
  ros::Publisher mapRegionPolygonPublisher_;

  geometry_msgs::PolygonStamped mapRegionPolygon_;
};

} /* namespace starleth_elevation_visualization */
