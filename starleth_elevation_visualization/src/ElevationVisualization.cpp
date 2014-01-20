/*
 * ElevationVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ElevationVisualization.hpp"

// StarlETH Navigation
#include "ElevationVisualizationHelpers.hpp"
#include <ElevationMessageHelpers.hpp>

// ROS
#include <geometry_msgs/Point.h>

// STD
#include <limits>
#include <algorithm>

using namespace std;
using namespace ros;
using namespace Eigen;

namespace starleth_elevation_visualization {

ElevationVisualization::ElevationVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      mapRegionVisualization_(nodeHandle_)
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
  nodeHandle_.param("elevation_map_topic", mapTopic_, string("/starleth_elevation_mapping/elevation_map"));
  nodeHandle_.param("marker_height", markerHeight_, 0.25);
  nodeHandle_.param("set_color_from_map", isSetColorFromMap_, false);
  nodeHandle_.param("set_color_from_variance", isSetColorFromVariance_, true);
  nodeHandle_.param("set_color_from_height", isSetColorFromHeight_, false);
  nodeHandle_.param("set_saturation_from_variance", isSetSaturationFromVariance_, false);
  nodeHandle_.param("set_alpha_from_variance", isSetAlphaFromVariance_, false);  // This looks bad in Rviz
  nodeHandle_.param("variance_lower_value_", varianceLowerValue_, pow(0.003, 2));
  nodeHandle_.param("variance_upper_value_", varianceUpperValue_, pow(0.01, 2));
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

  mapRegionVisualization_.initialize();

  ROS_INFO("StarlETH elevation visualization initialized.");
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

  if (!mapRegionVisualization_.update(map))
  {
    ROS_ERROR("ElevationVisualization: Generating map region visualization failed.");
    return;
  }

  mapMarkerArrayPublisher_.publish(mapMarkerArrayMessage_);
  mapRegionVisualization_.publish();
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

  float markerHeightOffset = static_cast<float>(markerHeight_/2.0);

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
      setColor(markerColor, elevation, variance, color);
      elevationMarker.colors.push_back(markerColor);
    }
  }

  return true;
}

bool ElevationVisualization::setColor(std_msgs::ColorRGBA& color, const double& elevation, const double& variance, const unsigned long& colorValue)
{
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 1.0;

  if (isSetColorFromMap_) setColorFromMap(color, colorValue);

  if (isSetColorFromVariance_)
  {
    setColorChannelFromVariance(color.r, variance);
    setColorChannelFromVariance(color.b, variance, true);
  }

  if (isSetColorFromHeight_)
  {
    setColorChannelFromVariance(color.r, variance);
    setColorChannelFromVariance(color.b, variance, true);
  }

  if (isSetSaturationFromVariance_) setSaturationFromVariance(color, variance);
  if (isSetAlphaFromVariance_) setColorChannelFromVariance(color.a, variance);

  return true;
}

bool ElevationVisualization::setColorFromMap(std_msgs::ColorRGBA& color, const unsigned long& colorValue)
{
  Vector3f colorVector;
  starleth_elevation_msg::copyColorValueToVector(colorValue, colorVector);
  getColorMessageFromColorVector(color, colorVector);
  return true;
}

bool ElevationVisualization::setColorChannelFromVariance(float& colorChannel, const double& variance, bool invert)
{
  double lowestVarianceValue = 0.0;
  double highestVarianceValue = 1.0;

  if (invert)
  {
    double tempValue = lowestVarianceValue;
    lowestVarianceValue = highestVarianceValue;
    highestVarianceValue = tempValue;
  }

  colorChannel = static_cast<float>(getValueFromVariance(lowestVarianceValue, highestVarianceValue, variance));
  return true;
}

bool ElevationVisualization::setSaturationFromVariance(std_msgs::ColorRGBA& color, const double& variance)
{
  const Eigen::Array3f HspFactors(.299, .587, .114); // see http://alienryderflex.com/hsp.html
  float saturationChange = static_cast<float>(getValueFromVariance(maxMarkerSaturation_, minMarkerSaturation_, variance));
  Vector3f colorVector;
  getColorVectorFromColorMessage(colorVector, color);
  float perceivedBrightness = sqrt((colorVector.array().square() * HspFactors).sum());
  colorVector = perceivedBrightness + saturationChange * (colorVector.array() - perceivedBrightness);
  colorVector = (colorVector.array().min(Array3f::Ones())).matrix();
  getColorMessageFromColorVector(color, colorVector);
  return true;
}

double ElevationVisualization::getValueFromVariance(const double& lowestVarianceValue, const double& highestVarianceValue, const double& variance)
{
  double m = (lowestVarianceValue - highestVarianceValue) / (varianceLowerValue_ - varianceUpperValue_);
  double b = highestVarianceValue - m * varianceUpperValue_;
  double value = m * variance + b;
  if (lowestVarianceValue < highestVarianceValue)
  {
    value = max(value, lowestVarianceValue);
    value = min(value, highestVarianceValue);
  }
  else
  {
    value = min(value, lowestVarianceValue);
    value = max(value, highestVarianceValue);
  }
  return value;
}

} /* namespace starleth_elevation_visualization */
