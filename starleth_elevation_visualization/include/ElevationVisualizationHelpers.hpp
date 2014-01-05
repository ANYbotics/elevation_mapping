/*
 * ElevationVisualizationHelpers.hpp
 *
 *  Created on: Jan 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Core>

namespace starleth_elevation_visualization {

inline void getColorMessageFromColorVector(std_msgs::ColorRGBA& colorMessage, const Eigen::Vector3f& colorVector)
{
  colorMessage.r = colorVector(0);
  colorMessage.g = colorVector(1);
  colorMessage.b = colorVector(2);
}

inline void getColorVectorFromColorMessage(Eigen::Vector3f& colorVector, const std_msgs::ColorRGBA& colorMessage)
{
  colorVector << colorMessage.r, colorMessage.g, colorMessage.b;
}

} /* namespace starleth_elevation_visualization */
