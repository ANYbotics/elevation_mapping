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

// STD
#include <limits>

using namespace std;
using namespace ros;
using namespace Eigen;

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
  nodeHandle_.param("variance_lower_value_", varianceLowerValue_, 0.001);
  nodeHandle_.param("variance_upper_value_", varianceUpperValue_, 0.2);
  nodeHandle_.param("min_marker_alpha", minMarkerAlpha_, 0.2);
  nodeHandle_.param("max_marker_alpha", maxMarkerAlpha_, 1.0);
  nodeHandle_.param("min_marker_saturation", minMarkerSaturation_, 0.0);
  nodeHandle_.param("max_marker_saturation", maxMarkerSaturation_, 1.0);

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
      const auto& elevation = map.elevation.data[dataIndex];
      const auto& variance = map.variance.data[dataIndex];
      const auto& color = map.color.data[dataIndex];

      // Do not continue for nan and inf values
      if (std::isnan(elevation) || std::isinf(variance)) continue;

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
      std_msgs::ColorRGBA markerColor;
      markerColor.b = 1.0;
      markerColor.a = 1.0;
      setColorFromMap(markerColor, color);
      setSaturationFromVariance(markerColor, variance);
//      setAlphaFromVariance(markerColor, variance); // This looks bad in Rviz
      elevationMarker.colors.push_back(markerColor);
    }
  }

  return true;
}

bool ElevationVisualization::setColorFromMap(std_msgs::ColorRGBA& color, const unsigned long& colorValue)
{
  Vector3f colorVector;
  starleth_elevation_msg::getColorVectorFromColorValue(colorVector, colorValue);
  getColorMessageFromColorVector(color, colorVector);
  return true;
}

bool ElevationVisualization::setAlphaFromVariance(std_msgs::ColorRGBA& color, const double& variance)
{
  color.a = getValueFromVariance(minMarkerAlpha_, maxMarkerAlpha_, variance);
  return true;
}

// Based on "changeSaturation" function by Darel Rex Finley
bool ElevationVisualization::setSaturationFromVariance(std_msgs::ColorRGBA& color, const double& variance)
{
  const Eigen::Array3f HspFactors(.299, .587, .114); // see http://alienryderflex.com/hsp.html
  float saturationChange = static_cast<float>(getValueFromVariance(minMarkerSaturation_, maxMarkerSaturation_, variance));
  Vector3f colorVector;
  getColorVectorFromColorMessage(colorVector, color);
  float perceivedBrightness = sqrt((colorVector.array().square() * HspFactors).sum());
  colorVector = perceivedBrightness + saturationChange * (colorVector.array() - perceivedBrightness);
  colorVector = (colorVector.array().min(Array3f::Ones())).matrix();
  getColorMessageFromColorVector(color, colorVector);
  return true;
}

double ElevationVisualization::getValueFromVariance(const double& minValue, const double& maxValue, const double& variance)
{
  double m = (maxValue - minValue) / (varianceLowerValue_ - varianceUpperValue_);
  double b = minValue - m * varianceUpperValue_;
  double value = m * variance + b;
  value = min(value, maxValue);
  value = max(value, minValue);
  return value;
}

} /* namespace starleth_elevation_visualization */
