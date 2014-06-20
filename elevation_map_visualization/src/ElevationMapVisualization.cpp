/*
 * ElevationMapVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_map_visualization/ElevationMapVisualization.hpp"

// Elevation Mapping
#include "elevation_map_visualization/ElevationMapVisualizationHelpers.hpp"
#include "elevation_map_visualization/VisualizationBase.hpp"
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"

// ROS
#include <geometry_msgs/Point.h>

// STD
#include <limits>
#include <algorithm>

using namespace std;
using namespace ros;
using namespace Eigen;

namespace elevation_map_visualization {

ElevationMapVisualization::ElevationMapVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      mapRegionVisualization_(nodeHandle_)
{
  ROS_INFO("Elevation map visualization node started.");
  readParameters();
  mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &ElevationMapVisualization::elevationMapCallback, this);
  mapMarkerArrayPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("elevation_map_marker_array", 1, true);
  initializeVisualization();
}

ElevationMapVisualization::~ElevationMapVisualization()
{

}

bool ElevationMapVisualization::readParameters()
{
  nodeHandle_.param("elevation_map_topic", mapTopic_, string("/elevation_mapping/elevation_map"));
  nodeHandle_.param("marker_height", markerHeight_, 0.25);
  nodeHandle_.param("sigma_bound", sigmaBound_, 0.0);
  nodeHandle_.param("set_color_from_map", isSetColorFromMap_, true);
  nodeHandle_.param("set_color_from_variance", isSetColorFromVariance_, false);
  nodeHandle_.param("set_color_from_height", isSetColorFromHeight_, false);
  nodeHandle_.param("set_saturation_from_variance", isSetSaturationFromVariance_, true);
  nodeHandle_.param("set_alpha_from_variance", isSetAlphaFromVariance_, false);  // This looks bad in Rviz
  nodeHandle_.param("show_empty_cells", showEmptyCells_, false);
  nodeHandle_.param("variance_lower_value_", varianceLowerValue_, pow(0.003, 2));
  nodeHandle_.param("variance_upper_value_", varianceUpperValue_, pow(0.01, 2));
  nodeHandle_.param("elevation_lower_value_", elevationLowerValue_, -0.1);
  nodeHandle_.param("elevation_upper_value_", elevationUpperValue_, 0.25);
  nodeHandle_.param("min_marker_alpha", minMarkerAlpha_, 0.2);
  nodeHandle_.param("max_marker_alpha", maxMarkerAlpha_, 1.0);
  nodeHandle_.param("min_marker_saturation", minMarkerSaturation_, 0.0);
  nodeHandle_.param("max_marker_saturation", maxMarkerSaturation_, 1.0);

  return true;
}

bool ElevationMapVisualization::initializeVisualization()
{
  mapMarkerArrayMessage_.markers.resize((int)VisualizationBase::MarkerTypes::Count);

  auto& elevationMarker = mapMarkerArrayMessage_.markers.at((int)VisualizationBase::MarkerTypes::Elevation);
  elevationMarker.id = (int)VisualizationBase::MarkerTypes::Elevation;
  elevationMarker.ns = "elevation";
  elevationMarker.lifetime = ros::Duration();
  elevationMarker.type = visualization_msgs::Marker::CUBE_LIST;
  elevationMarker.scale.z = markerHeight_;
  elevationMarker.action = visualization_msgs::Marker::ADD;

  mapRegionVisualization_.initialize();

  visualization_msgs::Marker& varianceMarker = mapMarkerArrayMessage_.markers.at((int)VisualizationBase::MarkerTypes::Variance);
  varianceVisualization_.initialize(&varianceMarker);

  ROS_INFO("Elevation map visualization initialized.");
  return true;
}

void ElevationMapVisualization::elevationMapCallback(
    const elevation_map_msg::ElevationMap& map)
{
  if (mapMarkerArrayPublisher_.getNumSubscribers () < 1) return;

  ROS_DEBUG("ElevationVisualization received an elevation map (time stamp %f) for visualization.", map.header.stamp.toSec());

  if (!generateVisualization(map))
  {
    ROS_ERROR("ElevationVisualization: Generating visualization failed.");
    return;
  }

  varianceVisualization_.generateVisualization(map);

  if (!mapRegionVisualization_.update(map))
  {
    ROS_ERROR("ElevationVisualization: Generating map region visualization failed.");
    return;
  }

  mapMarkerArrayPublisher_.publish(mapMarkerArrayMessage_);
  mapRegionVisualization_.publish();
}

bool ElevationMapVisualization::generateVisualization(
    const elevation_map_msg::ElevationMap& map)
{
  unsigned int nCells = map.elevation.layout.dim.at(0).stride;
  ROS_DEBUG("ElevationVisualization: Elevation data has has %i cells.", nCells);

  // Set marker info for elevation.
  auto& elevationMarker = mapMarkerArrayMessage_.markers.at((int)VisualizationBase::MarkerTypes::Elevation);
  elevationMarker.header = map.header;
  elevationMarker.scale.x = map.resolution;
  elevationMarker.scale.y = map.resolution;

  // Clear points.
  elevationMarker.points.clear();
  elevationMarker.colors.clear();

  float markerHeightOffset = static_cast<float>(markerHeight_/2.0);

  for (unsigned int i = 0; i < elevation_map_msg::getRows(map.elevation); ++i)
  {
    for (unsigned int j = 0; j < elevation_map_msg::getCols(map.elevation); ++j)
    {
      // Getting elevation value
      Vector2i cellIndex(i, j);
      unsigned int dataIndex = elevation_map_msg::get1dIndexFrom2dIndex(cellIndex, map);
      const auto& elevation = map.elevation.data[dataIndex];
      const auto& variance = map.variance.data[dataIndex];
      const auto& color = map.color.data[dataIndex];
      bool isEmtpyCell = false;

      if (std::isnan(elevation) || std::isinf(variance))
      {
        // Do not continue for nan and inf values
        if (!showEmptyCells_) continue;
        isEmtpyCell = true;
      }

      // Getting position of cell
      Vector2d position;
      elevation_map_msg::getPositionFromIndex(position, Array2i(i, j), map);

      // Add marker point
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = !isEmtpyCell ? (elevation + sigmaBound_ * sqrt(variance)  - markerHeightOffset) : 0;
      elevationMarker.points.push_back(point);

      // Add marker color
      std_msgs::ColorRGBA markerColor;
      setColor(markerColor, elevation, variance, color, isEmtpyCell);
      elevationMarker.colors.push_back(markerColor);
    }
  }

  return true;
}

bool ElevationMapVisualization::setColor(std_msgs::ColorRGBA& color, const double& elevation, const double& variance, const unsigned long& colorValue, bool isEmtpyCell)
{
  color.r = 0.0;
  color.g = 0.0;
  color.b = 1.0;
  color.a = 1.0;

  if (isEmtpyCell)
  {
    color.r = 0.6;
    color.g = 0.0;
    color.b = 0.0;
    return true;
  }

  if (isSetColorFromMap_) setColorFromMap(color, colorValue);

  if (isSetColorFromVariance_)
  {
    setColorChannelFromVariance(color.r, variance);
    setColorChannelFromVariance(color.b, variance, true);
  }

  if (isSetColorFromHeight_)
  {
    setColorFromHeight(color, elevation);
  }

  if (isSetSaturationFromVariance_) setSaturationFromVariance(color, variance);
  if (isSetAlphaFromVariance_) setColorChannelFromVariance(color.a, variance);

  return true;
}

bool ElevationMapVisualization::setColorFromMap(std_msgs::ColorRGBA& color, const unsigned long& colorValue)
{
  Vector3f colorVector;
  elevation_map_msg::copyColorValueToVector(colorValue, colorVector);
  getColorMessageFromColorVector(color, colorVector);
  return true;
}

bool ElevationMapVisualization::setColorChannelFromVariance(float& colorChannel, const double& variance, bool invert)
{
  double lowestVarianceValue = 0.0;
  double highestVarianceValue = 1.0;

  if (invert)
  {
    double tempValue = lowestVarianceValue;
    lowestVarianceValue = highestVarianceValue;
    highestVarianceValue = tempValue;
  }

  colorChannel = static_cast<float>(computeLinearMapping(variance, varianceLowerValue_, varianceUpperValue_, lowestVarianceValue, highestVarianceValue));


  return true;
}

bool ElevationMapVisualization::setSaturationFromVariance(std_msgs::ColorRGBA& color, const double& variance)
{
  const Eigen::Array3f HspFactors(.299, .587, .114); // see http://alienryderflex.com/hsp.html
  float saturationChange = static_cast<float>(computeLinearMapping(variance, varianceLowerValue_, varianceUpperValue_, maxMarkerSaturation_, minMarkerSaturation_));
  Vector3f colorVector;
  getColorVectorFromColorMessage(colorVector, color);
  float perceivedBrightness = sqrt((colorVector.array().square() * HspFactors).sum());
  colorVector = perceivedBrightness + saturationChange * (colorVector.array() - perceivedBrightness);
  colorVector = (colorVector.array().min(Array3f::Ones())).matrix();
  getColorMessageFromColorVector(color, colorVector);
  return true;
}

bool ElevationMapVisualization::setColorFromHeight(std_msgs::ColorRGBA& color, const double& height)
{
  Vector3f hsl; // Hue: [0, 2 Pi], Saturation and Lightness: [0, 1]
  Vector3f rgb;

  hsl[0] = static_cast<float>(computeLinearMapping(height, elevationLowerValue_, elevationUpperValue_, 0.0, 2.0 * M_PI));
  hsl[1] = 1.0;
  hsl[2] = 1.0;

  float offset = 2.0 / 3.0 * M_PI;
  Array3f rgbOffset(0, -offset, offset);
  rgb = ((rgbOffset + hsl[0]).cos() + 0.5).min(Array3f::Ones()).max(Array3f::Zero()) * hsl[2];
  float white = Vector3f(0.3, 0.59, 0.11).transpose() * rgb;
  float saturation = 1.0 - hsl[1];
  rgb = rgb + ((-rgb.array() + white) * saturation).matrix();

  getColorMessageFromColorVector(color, rgb);
  return true;
}

double ElevationMapVisualization::computeLinearMapping(
    const double& sourceValue, const double& sourceLowerValue, const double& sourceUpperValue,
    const double& mapLowerValue, const double& mapUpperValue)
{
  double m = (mapLowerValue - mapUpperValue) / (sourceLowerValue - sourceUpperValue);
  double b = mapUpperValue - m * sourceUpperValue;
  double mapValue = m * sourceValue + b;
  if (mapLowerValue < mapUpperValue)
  {
    mapValue = max(mapValue, mapLowerValue);
    mapValue = min(mapValue, mapUpperValue);
  }
  else
  {
    mapValue = min(mapValue, mapLowerValue);
    mapValue = max(mapValue, mapUpperValue);
  }
  return mapValue;
}

} /* namespace */
