/*
 * ElevationVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ElevationVisualization.hpp"

// StarlETH Navigation
#include <ElevationMapHelpers.hpp>

// std::min, std::max
#include <algorithm>

using namespace std;
using namespace ros;
using namespace Eigen;

namespace starleth_elevation_visualization {

ElevationVisualization::ElevationVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("StarlETH elevation visualization node started.");
  readParameters();
  mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &ElevationVisualization::elevationMapCallback, this);
  mapMarkerArrayPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("elevation_map_marker_array", 1, true);
  initializeVisualization();
}

ElevationVisualization::~ElevationVisualization()
{
  // TODO Auto-generated destructor stub
}

bool ElevationVisualization::readParameters()
{
  nodeHandle_.param("elevation_map_topic", mapTopic_, string("/elevation_map"));
  nodeHandle_.param("marker_height", markerHeight_, 0.25);
  nodeHandle_.param("min_marker_alpha", minMarkerAlpha_, 0.2);
  nodeHandle_.param("max_marker_alpha", maxMarkerAlpha_,1.0);
  nodeHandle_.param("variance_lower_value_", varianceLowerValue_, 0.01);
  nodeHandle_.param("variance_upper_value_", varianceUpperValue_, 0.2);
  return true;
}

bool ElevationVisualization::initializeVisualization()
{
  mapMarkerArrayMessage_.markers.resize((int)MarkerTypes::Count); // Could add more marker types here

  auto& elevationMarker = mapMarkerArrayMessage_.markers.at((int)MarkerTypes::ElevationMap);
  elevationMarker.id = (int)MarkerTypes::ElevationMap;
  elevationMarker.ns = "elevation_map";
  elevationMarker.lifetime = ros::Duration();
  elevationMarker.type = visualization_msgs::Marker::CUBE_LIST;
  elevationMarker.scale.z = markerHeight_;
  elevationMarker.action = visualization_msgs::Marker::ADD;

  return true;
}

void ElevationVisualization::elevationMapCallback(
    const starleth_elevation_msg::ElevationMap& map)
{
  if (mapMarkerArrayPublisher_.getNumSubscribers () < 1) return;

  ROS_DEBUG("ElevationVisualization received an elevation map (time stamp %f) for visualization.", map.header.stamp.toSec());

  if (!generateVisualization(map))
  {
    ROS_ERROR("ElevationVisualization: Generating visualization failed.");
    return;
  }

  mapMarkerArrayPublisher_.publish(mapMarkerArrayMessage_);
}

bool ElevationVisualization::generateVisualization(
    const starleth_elevation_msg::ElevationMap& map)
{
  unsigned int nCells = map.elevation.layout.dim.at(0).stride;
  ROS_DEBUG("ElevationVisualization: Elevation data has has %i cells.", nCells);

  // Set marker info for elevation map
  auto& elevationMarker = mapMarkerArrayMessage_.markers.at((int)MarkerTypes::ElevationMap);
  elevationMarker.header = map.header;
  elevationMarker.scale.x = map.resolution;
  elevationMarker.scale.y = map.resolution;

  // Clear points
  elevationMarker.points.clear();
  elevationMarker.colors.clear();

  double markerHeightOffset = markerHeight_/2.0;

  for (unsigned int i = 0; i < starleth_elevation_msg::getRows(map.elevation); ++i)
  {
    for (unsigned int j = 0; j < starleth_elevation_msg::getCols(map.elevation); ++j)
    {
      // Getting elevation value
      Vector2i cellIndex(i, j);
      unsigned int dataIndex = starleth_elevation_msg::get1dIndexFrom2dIndex(cellIndex, map);
      double elevation = map.elevation.data[dataIndex];
      double variance = map.variance.data[dataIndex];

      // Do not continue for nan values
      if (std::isnan(elevation)) continue;

      // Getting position of cell
      Vector2d position;
      starleth_elevation_msg::getPositionFromIndex(position, Array2i(i, j), map);

      // Add marker point
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = elevation - markerHeightOffset;
      elevationMarker.points.push_back(point);

      // Add marker color
      std_msgs::ColorRGBA color;
      color.r = 0.0;
      color.g = 0.3;
      color.b = 1.0;
      color.a = 1.0;

      setColorFromVariance(color, variance);

      elevationMarker.colors.push_back(color);
    }
  }

  return true;
}

bool ElevationVisualization::setAlphaFromVariance(std_msgs::ColorRGBA& color, const double& variance)
{
  double m = (maxMarkerAlpha_ - minMarkerAlpha_) / (varianceLowerValue_ - varianceUpperValue_);
  double b = minMarkerAlpha_ - m * varianceUpperValue_;
  double alpha = m * variance + b;
  alpha = min(alpha, maxMarkerAlpha_);
  alpha = max(alpha, minMarkerAlpha_);
  color.a = alpha;
}

bool ElevationVisualization::setColorFromVariance(std_msgs::ColorRGBA& color, const double& variance)
{
  double m = (maxMarkerAlpha_ - minMarkerAlpha_) / (varianceLowerValue_ - varianceUpperValue_);
  double b = minMarkerAlpha_ - m * varianceUpperValue_;
  double blue = m * variance + b;
  blue = min(blue, maxMarkerAlpha_);
  blue = max(blue, minMarkerAlpha_);
  color.b = blue;
}

} /* namespace starleth_elevation_visualization */
