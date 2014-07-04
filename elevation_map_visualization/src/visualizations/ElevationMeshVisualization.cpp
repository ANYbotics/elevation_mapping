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
#include <geometric_shapes/shape_operations.h>
#include <shape_tools/shape_to_marker.h>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <shape_msgs/Mesh.h>

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
  int baseColorValue;
  nodeHandle_.param("elevation/base_color", baseColorValue, 1323750); // blue, http://www.wolframalpha.com/input/?i=BitOr%5BBitShiftLeft%5Br%2C16%5D%2C+BitShiftLeft%5Bg%2C8%5D%2C+b%5D+where+%7Br%3D20%2C+g%3D50%2C+b%3D230%7D
  setColorFromColorValue(baseColor_, baseColorValue, true);

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
  EigenSTL::vector_Vector3d vertices;

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

      if (!elevation_map_msg::isValidCell(elevation, variance))
      {
        continue;
      }

      // Getting position of cell.
      Vector2d position;
      elevation_map_msg::getPositionFromIndex(position, Array2i(i, j), map);

      Eigen::Vector3d vertice;
      vertice.head(2) = position;
      vertice.z() = elevation;

      vertices.push_back(vertice);

//      // Add marker point.
//      geometry_msgs::Point point;
//      point.x = position.x();
//      point.y = position.y();
//      point.z = !isEmtpyCell ? (elevation - markerHeightOffset) : -markerHeightOffset;
//      marker_->points.push_back(point);
//
      // Add marker color.
      std_msgs::ColorRGBA markerColor;
      setColor(markerColor, elevation, variance, color, false);
      marker_->colors.push_back(markerColor);
    }
  }

  shapes::Mesh* mesh = shapes::createMeshFromVertices(vertices);
  shape_msgs::Mesh coMesh;
  shapes::ShapeMsg coMeshMsg;
  shapes::constructMsgFromShape(mesh, coMeshMsg);
  coMesh = boost::get<shape_msgs::Mesh>(coMeshMsg);
  shape_tools::constructMarkerFromShape(coMesh, *marker_, true);

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
