/*
 * ElevationVisualization.cpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ElevationVisualization.hpp"

// StarlETH Navigation
#include <ElevationMapTransformations.hpp>

using namespace std;
using namespace ros;
using namespace Eigen;

namespace starleth_elevation_visualization {

ElevationVisualization::ElevationVisualization(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
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

  for (unsigned int i = 0; i < (int) map.elevation.layout.dim.at(0).size; ++i)
  {
    for (unsigned int j = 0; j < (int) map.elevation.layout.dim.at(1).size; ++j)
    {
      // Getting elevation value
      Vector2i cellIndex(i, j);
      unsigned int elevationDataIndex = starleth_elevation_msg::get1dIndexFrom2dIndex(cellIndex, map);
      double elevation = map.elevation.data[elevationDataIndex];

      // Do not continue for nan values
      if (std::isnan(elevation))
      {
        continue;
      }

      // Getting position of cell
      Vector2d position;
      starleth_elevation_msg::getPositionFromIndex(position, Vector2i(i, j), map);

      // Add marker point
      geometry_msgs::Point point;
      point.x = position.x();
      point.y = position.y();
      point.z = elevation;
      elevationMarker.points.push_back(point);

      // Add marker color
      std_msgs::ColorRGBA color;
      color.r = 0.0;
      color.g = 0.3;
      color.b = 1.0;
      color.a = 1.0;
      elevationMarker.colors.push_back(color);
    }
  }

  return true;
}

} /* namespace starleth_elevation_visualization */
