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
  bool initializeVisualization();
  bool generateVisualization(const starleth_elevation_msg::ElevationMap& map);
  bool setAlphaFromVariance(std_msgs::ColorRGBA& color, const double& variance);
  bool setColorFromVariance(std_msgs::ColorRGBA& color, const double& variance);

  enum class MarkerTypes
  {
    ElevationMap,
    Count
  };

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber mapSubscriber_;
  ros::Publisher mapMarkerArrayPublisher_;
  visualization_msgs::MarkerArray mapMarkerArrayMessage_;

  std::string mapTopic_;
  double markerHeight_;
  double minMarkerAlpha_;
  double maxMarkerAlpha_;
  double varianceLowerValue_;
  double varianceUpperValue_;
};

} /* namespace starleth_elevation_visualization */
