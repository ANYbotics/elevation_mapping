/*
 * ElevationVisualization.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// StarlETH Navigation
#include <starleth_elevation_msg/ElevationMap.h>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

namespace starleth_elevation_visualization {

/*
 *
 */
class ElevationVisualization
{
 public:
  ElevationVisualization(ros::NodeHandle& nodeHandle);

  virtual ~ElevationVisualization();

  void elevationMapCallback(const starleth_elevation_msg::ElevationMap& map);

 private:
  bool readParameters();
  bool generateVisualization(const starleth_elevation_msg::ElevationMap& map);

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber mapSubscriber_;
  ros::Publisher mapMarkerArrayPublisher_;
  visualization_msgs::MarkerArray mapMarkerArrayMessage_;
  std::string mapTopic_;
  int nCellsLastTime_;

};

} /* namespace starleth_elevation_visualization */
