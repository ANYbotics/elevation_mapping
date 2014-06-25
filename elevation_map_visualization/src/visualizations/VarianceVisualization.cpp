/*
 * VarianceVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_map_visualization/visualizations/VarianceVisualization.hpp"

// Elevation Mapping
#include "elevation_map_visualization/ElevationMapVisualizationHelpers.hpp"
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"

// Eigen
#include <Eigen/Core>

namespace elevation_map_visualization {

VarianceVisualization::VarianceVisualization(ros::NodeHandle& nodeHandle)
    : VisualizationBase(nodeHandle)
{

}

VarianceVisualization::~VarianceVisualization()
{

}

bool VarianceVisualization::readParameters()
{
  nodeHandle_.param("variance/sigma_bound", sigmaBound_, 1.0);
  nodeHandle_.param("variance/marker_size", markerSize_, 0.003);
  nodeHandle_.param("variance/set_color_from_variance", isSetColorFromVariance_, false);
  nodeHandle_.param("variance/variance_lower_value", varianceLowerValue_, pow(0.0, 2));
  nodeHandle_.param("variance/variance_upper_value", varianceUpperValue_, pow(0.03, 2));

  int baseColorValue;
  nodeHandle_.param("variance/base_color", baseColorValue, 16711680); // red, http://www.wolframalpha.com/input/?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D255%2C+g%3D0%2C+b%3D0%7D
  setColorFromColorValue(baseColor_, baseColorValue, true);

  int lowerVarianceColorValue;
  nodeHandle_.param("variance/lower_variance_color", lowerVarianceColorValue, 255);
  setColorFromColorValue(lowerVarianceColor_, lowerVarianceColorValue, true);

  int upperVarianceColorValue;
  nodeHandle_.param("variance/upper_variance_color", upperVarianceColorValue, 16711680);
  setColorFromColorValue(upperVarianceColor_, upperVarianceColorValue, true);

  return true;
}

bool VarianceVisualization::initialize(visualization_msgs::Marker* marker)
{
  if(!VisualizationBase::initialize(marker)) return false;
  marker_->ns = "variance";
  marker_->lifetime = ros::Duration();
  marker_->action = visualization_msgs::Marker::ADD;
  marker_->type = visualization_msgs::Marker::SPHERE_LIST;
  marker_->scale.x = markerSize_;
  marker_->scale.y = markerSize_;
  marker_->scale.z = markerSize_;
  return true;
}

bool VarianceVisualization::generateVisualization(
    const elevation_map_msg::ElevationMap& map)
{
  // Set marker info for variance.
  marker_->header = map.header;

  // Clear points.
  marker_->points.clear();
  marker_->colors.clear();

  for (unsigned int i = 0; i < elevation_map_msg::getRows(map.elevation); ++i)
  {
    for (unsigned int j = 0; j < elevation_map_msg::getCols(map.elevation); ++j)
    {
      // Getting map data.
      Vector2i cellIndex(i, j);
      unsigned int dataIndex = elevation_map_msg::get1dIndexFrom2dIndex(cellIndex, map);
      const auto& elevation = map.elevation.data[dataIndex];
      const auto& variance = map.variance.data[dataIndex];

      if (!elevation_map_msg::isValidCell(elevation, variance)) continue;

      // Getting position of cell.
      Vector2d position;
      elevation_map_msg::getPositionFromIndex(position, Array2i(i, j), map);

      // Add marker point.
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = elevation + sigmaBound_ * sqrt(variance);
      marker_->points.push_back(point);

      // Add marker color.
      std_msgs::ColorRGBA markerColor;
      setColor(markerColor, variance);
      marker_->colors.push_back(markerColor);
    }
  }

  return true;
}

void VarianceVisualization::setColor(std_msgs::ColorRGBA& color, const double variance)
{
  color = baseColor_;

  if (isSetColorFromVariance_) interpolateBetweenColors(color, lowerVarianceColor_, upperVarianceColor_, variance, varianceLowerValue_, varianceUpperValue_);
}

} /* namespace elevation_map_visualization */
