/*
 * VarianceVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_map_visualization/VarianceVisualization.hpp"

// Elevation Mapping
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"

namespace elevation_map_visualization {

VarianceVisualization::VarianceVisualization()
{
  sigmaBound_ = 1.0; // TODO
}

VarianceVisualization::~VarianceVisualization()
{

}

bool VarianceVisualization::initialize(visualization_msgs::Marker* marker)
{
  if(!VisualizationBase::initialize(marker)) return false;
  marker_->ns = "variance";
  marker_->type = visualization_msgs::Marker::SPHERE_LIST;
  marker_->scale.x = 0.003;
  marker_->scale.y = 0.003;
  marker_->scale.z = 0.003;
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
      Eigen::Vector2i cellIndex(i, j);
      unsigned int dataIndex = elevation_map_msg::get1dIndexFrom2dIndex(cellIndex, map);
      const auto& elevation = map.elevation.data[dataIndex];
      const auto& variance = map.variance.data[dataIndex];
      const auto& color = map.color.data[dataIndex];

      if (std::isnan(elevation) || std::isinf(variance)) // TODO replace with generic function.
      {
        // Do not continue for nan and inf values.
        continue;
      }

      // Getting position of cell.
      Vector2d position;
      elevation_map_msg::getPositionFromIndex(position, Array2i(i, j), map);

      // Add marker point.
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = elevation + sigmaBound_ * sqrt(variance);
      marker_->points.push_back(point);

      // Add marker color
      std_msgs::ColorRGBA markerColor;
//      setColor(markerColor, elevation, variance, color, isEmtpyCell); // TODO
      markerColor.r = 1.0;
      markerColor.g = 0.0;
      markerColor.b = 0.0;
      markerColor.a = 1.0;

      marker_->colors.push_back(markerColor);
    }
  }

  return true;
}

} /* namespace elevation_map_visualization */
