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
  nCellsLastTime_ = 0;
}

ElevationVisualization::~ElevationVisualization()
{
  // TODO Auto-generated destructor stub
}

bool ElevationVisualization::readParameters()
{
  nodeHandle_.param("elevation_map_topic", mapTopic_, string("/elevation_map"));
  return true;
}

void ElevationVisualization::elevationMapCallback(
    const starleth_elevation_msg::ElevationMap& map)
{
//  if (elevationMapMarkerArrayPublisher_.getNumSubscribers () < 1) return;

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
  unsigned int nCells = map.elevation.layout.dim[0].stride;

  ROS_DEBUG("ElevationVisualization: Elevation data has has %i cells.", nCells);

  if (nCells != nCellsLastTime_)
  {
    mapMarkerArrayMessage_.markers.resize(nCells);
  }

  int markerIndex = -1;
  for (unsigned int i = 0; i < (int) map.elevation.layout.dim[0].size; ++i)
  {
    for (unsigned int j = 0; j < (int) map.elevation.layout.dim[1].size; ++j)
    {
      markerIndex++;

      // Getting elevation value
      Vector2i cellIndex(i, j);
      unsigned int n = starleth_elevation_msg::get1dIndexFrom2dIndex(cellIndex, map);
      double elevation = map.elevation.data[n];

      // Do not continue for nan values
      if (std::isnan(elevation))
      {
        mapMarkerArrayMessage_.markers[markerIndex].action = visualization_msgs::Marker::DELETE;
        mapMarkerArrayMessage_.markers[markerIndex].header = map.header;
        mapMarkerArrayMessage_.markers[markerIndex].id = markerIndex;
        continue;
      }

      // Getting position of cell
      Vector2d position;
      starleth_elevation_msg::getPositionFromIndex(position, Vector2i(i, j), map);

      // Setting up marker
      mapMarkerArrayMessage_.markers[markerIndex].pose.position.x = position.x();
      mapMarkerArrayMessage_.markers[markerIndex].pose.position.y = position.y();
      mapMarkerArrayMessage_.markers[markerIndex].pose.position.z = elevation;

//      geometry_msgs::Point position2;
//      position2.x = position.x();;
//      position2.y = position.y();
//      position2.z = elevation;
//
//      mapMarkerArrayMessage_.markers[markerIndex].points.push_back(position2);

      mapMarkerArrayMessage_.markers[markerIndex].header = map.header;
      mapMarkerArrayMessage_.markers[markerIndex].id = markerIndex;
      mapMarkerArrayMessage_.markers[markerIndex].ns = "elevation_map";
      mapMarkerArrayMessage_.markers[markerIndex].color.r = 0.5f;
      mapMarkerArrayMessage_.markers[markerIndex].color.g = 0.5f;
      mapMarkerArrayMessage_.markers[markerIndex].color.b = 1.0f;
      mapMarkerArrayMessage_.markers[markerIndex].color.a = 1.0f;
      mapMarkerArrayMessage_.markers[markerIndex].lifetime = ros::Duration();
      mapMarkerArrayMessage_.markers[markerIndex].type = visualization_msgs::Marker::CUBE_LIST;
      mapMarkerArrayMessage_.markers[markerIndex].scale.x = map.resolution;
      mapMarkerArrayMessage_.markers[markerIndex].scale.y = map.resolution;
      mapMarkerArrayMessage_.markers[markerIndex].scale.z = 1.0;
      mapMarkerArrayMessage_.markers[markerIndex].action = visualization_msgs::Marker::ADD;
    }
  }

//
//  // visualize only known cells
//  if(elevation_map.data[MAP_IDX(elevation_map.info.width, index_x, index_y)] != (int16_t)-elevation_map.info.zero_elevation)
//  {
//      geometry_msgs::Point cube_center;
//      Eigen::Vector2f index_map(index_x, index_y);
//      Eigen::Vector2f index_world = world_map_transform.getC1Coords(index_map);
//
//      cube_center.x = index_world(0);//+elevation_map.info.resolution_xy/2.0;
//      cube_center.y = index_world(1);//+elevation_map.info.resolution_xy/2.0;
//      cube_center.z = (elevation_map.data[MAP_IDX(elevation_map.info.width, index_x, index_y)]-elevation_map.info.zero_elevation)*elevation_map.info.resolution_z;
//      current_height_level = max_height_levels/2+(int)round(std::min(std::max((double)cube_center.z+local_map_transform.getOrigin().z(), -(double)max_height), (double)max_height)*(double)max_height_levels/((double)max_height*2.0f));
//      map_marker_array_msg.markers[current_height_level].points.push_back(cube_center);
//
//      if(use_color_map)
//      {
//          double h = (1.0 - std::min(std::max((cube_center.z-min_height)/ (max_height - min_height), 0.0), 1.0)) *color_factor;
//          map_marker_array_msg.markers[current_height_level].colors.push_back(heightMapColor(h));
//      }
//  }

  if (nCellsLastTime_ > nCells)
  {
    for (unsigned int i = nCells; i < nCellsLastTime_; ++i)
    {
      mapMarkerArrayMessage_.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }

  nCellsLastTime_ = nCells;



  return true;

}

} /* namespace starleth_elevation_visualization */
