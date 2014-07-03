/*
 * ElevationMeshVisualization.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_map_visualization/visualizations/ElevationMeshVisualization.hpp"

// Elevation Mapping
#include "elevation_map_visualization/ElevationMapVisualizationHelpers.hpp"
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"

// Eigen
#include <Eigen/Core>

// ROS
#include <geometric_shapes/mesh_operations.h>
#include <eigen_stl_containers/eigen_stl_containers.h>

using namespace Eigen;

namespace elevation_map_visualization {

ElevationMeshVisualization::ElevationMeshVisualization(ros::NodeHandle& nodeHandle)
    : VisualizationBase(nodeHandle)
{

}

ElevationMeshVisualization::~ElevationMeshVisualization()
{

}

bool ElevationMeshVisualization::readParameters()
{
  return true;
}

bool ElevationMeshVisualization::initialize(visualization_msgs::Marker* marker)
{
  if(!VisualizationBase::initialize(marker)) return false;
  marker_->ns = "elevation_mesh";
  marker_->lifetime = ros::Duration();
  marker_->action = visualization_msgs::Marker::ADD;
  return true;
}

bool ElevationMeshVisualization::generateVisualization(
    const elevation_map_msg::ElevationMap& map)
{
  // Set marker info.
  marker_->header = map.header;

  // Clear points.
  marker_->points.clear();
  marker_->colors.clear();

  // Generate mesh.
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

//      // Add marker point.
//      geometry_msgs::Point point;
//      point.x = position.x();
//      point.y = position.y();
//      point.z = !isEmtpyCell ? (elevation - markerHeightOffset) : -markerHeightOffset;
//      marker_->points.push_back(point);
//
//      // Add marker color.
//      std_msgs::ColorRGBA markerColor;
//      setColor(markerColor, elevation, variance, color, isEmtpyCell);
//      marker_->colors.push_back(markerColor);
    }
  }

  EigenSTL::vector_Vector3d vertices;

  shapes::createMeshFromVertices(vertices);

  //shape_tools::constructMarkerFromShape(const shape_msgs::Mesh &shape_msg, visualization_msgs::Marker &mk, bool use_mesh_triangle_list);

  return true;
}

void ElevationMeshVisualization::setColor(std_msgs::ColorRGBA& color, const double elevation, const double variance, const unsigned long colorValue, bool isEmtpyCell)
{
  color = baseColor_;

//  if (isEmtpyCell)
//  {
//    color = emptyCellColor_;
//    return;
//  }
//
//  if (isSetColorFromMap_)
//  {
//    setColorFromColorValue(color, colorValue, true);
//  }
//  else if (isSetColorFromVariance_)
//  {
//    interpolateBetweenColors(color, lowerVarianceColor_, upperVarianceColor_, variance, varianceLowerValue_, varianceUpperValue_);
//  }
//  else if (isSetColorFromHeight_)
//  {
//    setColorFromValue(color, elevation, elevationLowerValue_, elevationUpperValue_);
//  }
//
//  if (isSetSaturationFromVariance_) setSaturationFromValue(color, variance, varianceLowerValue_, varianceUpperValue_, maxMarkerSaturation_, minMarkerSaturation_);
//  if (isSetAlphaFromVariance_) setColorChannelFromValue(color.a, variance, elevationLowerValue_, elevationUpperValue_);
}

} /* namespace elevation_map_visualization */
