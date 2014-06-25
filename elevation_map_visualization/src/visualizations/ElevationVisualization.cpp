/*
 * ElevationVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_map_visualization/visualizations/ElevationVisualization.hpp"

// Elevation Mapping
#include "elevation_map_visualization/ElevationMapVisualizationHelpers.hpp"
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"

// Eigen
#include <Eigen/Core>

namespace elevation_map_visualization {

ElevationVisualization::ElevationVisualization(ros::NodeHandle& nodeHandle)
    : VisualizationBase(nodeHandle)
{

}

ElevationVisualization::~ElevationVisualization()
{

}

bool ElevationVisualization::readParameters()
{
  nodeHandle_.param("elevation/marker_height", markerHeight_, 0.25);
  nodeHandle_.param("elevation/show_empty_cells", showEmptyCells_, false);
  nodeHandle_.param("elevation/set_color_from_map", isSetColorFromMap_, true);
  nodeHandle_.param("elevation/set_color_from_variance", isSetColorFromVariance_, false);
  nodeHandle_.param("elevation/set_color_from_height", isSetColorFromHeight_, false);
  nodeHandle_.param("elevation/set_saturation_from_variance", isSetSaturationFromVariance_, true);
  nodeHandle_.param("elevation/set_alpha_from_variance", isSetAlphaFromVariance_, false);
  nodeHandle_.param("elevation/variance_lower_value", varianceLowerValue_, pow(0.0, 2));
  nodeHandle_.param("elevation/variance_upper_value", varianceUpperValue_, pow(0.03, 2));
  nodeHandle_.param("elevation/elevation_lower_value", elevationLowerValue_, -0.5);
  nodeHandle_.param("elevation/elevation_upper_value", elevationUpperValue_, 0.5);
  nodeHandle_.param("elevation/min_marker_saturation", minMarkerSaturation_, 0.0);
  nodeHandle_.param("elevation/max_marker_saturation", maxMarkerSaturation_, 1.0);
  nodeHandle_.param("elevation/min_marker_alpha", minMarkerAlpha_, 0.2);
  nodeHandle_.param("elevation/max_marker_alpha", maxMarkerAlpha_, 1.0);

  int baseColorValue;
  nodeHandle_.param("elevation/base_color", baseColorValue, 1323750); // blue, http://www.wolframalpha.com/input/?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D20%2C+g%3D50%2C+b%3D230%7D
  setColorFromColorValue(baseColor_, baseColorValue, true);

  int emptyCellColorValue;
  nodeHandle_.param("elevation/empty_cell_color", emptyCellColorValue, 5263440); // gray, http://www.wolframalpha.com/input/?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D80%2C+g%3D80%2C+b%3D80%7D
  setColorFromColorValue(emptyCellColor_, emptyCellColorValue, true);

  int lowerVarianceColorValue;
  nodeHandle_.param("elevation/lower_variance_color", lowerVarianceColorValue, 0);
  setColorFromColorValue(lowerVarianceColor_, lowerVarianceColorValue, true);

  int upperVarianceColorValue;
  nodeHandle_.param("elevation/upper_variance_color", upperVarianceColorValue, 16711680);
  setColorFromColorValue(upperVarianceColor_, upperVarianceColorValue, true);

  return true;
}

bool ElevationVisualization::initialize(visualization_msgs::Marker* marker)
{
  if(!VisualizationBase::initialize(marker)) return false;
  marker_->ns = "elevation";
  marker_->lifetime = ros::Duration();
  marker_->action = visualization_msgs::Marker::ADD;
  marker_->type = visualization_msgs::Marker::CUBE_LIST;
  marker_->scale.z = markerHeight_;
  return true;
}

bool ElevationVisualization::generateVisualization(
    const elevation_map_msg::ElevationMap& map)
{
  // Set marker info.
  marker_->header = map.header;
  marker_->scale.x = map.resolution;
  marker_->scale.y = map.resolution;

  // Clear points.
  marker_->points.clear();
  marker_->colors.clear();

  float markerHeightOffset = static_cast<float>(markerHeight_/2.0);

  for (unsigned int i = 0; i < elevation_map_msg::getRows(map.elevation); ++i)
  {
    for (unsigned int j = 0; j < elevation_map_msg::getCols(map.elevation); ++j)
    {
      // Getting map data.
      Vector2i cellIndex(i, j);
      unsigned int dataIndex = elevation_map_msg::get1dIndexFrom2dIndex(cellIndex, map);
      const auto& elevation = map.elevation.data[dataIndex];
      const auto& variance = map.variance.data[dataIndex];
      const auto& color = map.color.data[dataIndex];
      bool isEmtpyCell = false;

      if (!elevation_map_msg::isValidCell(elevation, variance))
      {
        if (!showEmptyCells_) continue;
        isEmtpyCell = true;
      }

      // Getting position of cell.
      Vector2d position;
      elevation_map_msg::getPositionFromIndex(position, Array2i(i, j), map);

      // Add marker point.
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = !isEmtpyCell ? (elevation - markerHeightOffset) : -markerHeightOffset;
      marker_->points.push_back(point);

      // Add marker color.
      std_msgs::ColorRGBA markerColor;
      setColor(markerColor, elevation, variance, color, isEmtpyCell);
      marker_->colors.push_back(markerColor);
    }
  }

  return true;
}

void ElevationVisualization::setColor(std_msgs::ColorRGBA& color, const double elevation, const double variance, const unsigned long colorValue, bool isEmtpyCell)
{
  color = baseColor_;

  if (isEmtpyCell)
  {
    color = emptyCellColor_;
    return;
  }

  if (isSetColorFromMap_)
  {
    setColorFromColorValue(color, colorValue, true);
  }
  else if (isSetColorFromVariance_)
  {
    interpolateBetweenColors(color, lowerVarianceColor_, upperVarianceColor_, variance, varianceLowerValue_, varianceUpperValue_);
  }
  else if (isSetColorFromHeight_)
  {
    setColorFromValue(color, elevation, elevationLowerValue_, elevationUpperValue_);
  }

  if (isSetSaturationFromVariance_) setSaturationFromValue(color, variance, varianceLowerValue_, varianceUpperValue_, maxMarkerSaturation_, minMarkerSaturation_);
  if (isSetAlphaFromVariance_) setColorChannelFromValue(color.a, variance, elevationLowerValue_, elevationUpperValue_);
}

} /* namespace elevation_map_visualization */
