/*
 * MapRegionVisualization.cpp
 *
 *  Created on: Jan 17, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_map_visualization/visualizations/MapRegionVisualization.hpp"

// ROS
#include <geometry_msgs/Point32.h>

namespace elevation_map_visualization {

MapRegionVisualization::MapRegionVisualization(ros::NodeHandle& nodeHandle)
: nodeHandle_(nodeHandle)
{
  mapRegionPolygonPublisher_ = nodeHandle_.advertise<geometry_msgs::PolygonStamped>("elevation_map_region", 1, true);
}

MapRegionVisualization::~MapRegionVisualization()
{

}

bool MapRegionVisualization::initialize()
{
  for (int i = 0; i < getNPoints(); i++)
  {
    geometry_msgs::Point32 point;
    point.z = 0.0;
    mapRegionPolygon_.polygon.points.push_back(point);
  }

  return true;
}

bool MapRegionVisualization::update(
    const elevation_map_msg::ElevationMap& map)
{
  mapRegionPolygon_.header = map.header;

  float halfLengthX = map.lengthInX / 2.0;
  float halfLengthY = map.lengthInY / 2.0;

  mapRegionPolygon_.polygon.points[0].x = map.position.x + halfLengthX;
  mapRegionPolygon_.polygon.points[0].y = map.position.y + halfLengthY;
  mapRegionPolygon_.polygon.points[1].x = map.position.x + halfLengthX;
  mapRegionPolygon_.polygon.points[1].y = map.position.y - halfLengthY;
  mapRegionPolygon_.polygon.points[2].x = map.position.x - halfLengthX;
  mapRegionPolygon_.polygon.points[2].y = map.position.y - halfLengthY;
  mapRegionPolygon_.polygon.points[3].x = map.position.x - halfLengthX;
  mapRegionPolygon_.polygon.points[3].y = map.position.y + halfLengthY;

  return true;
}

bool MapRegionVisualization::publish()
{
  mapRegionPolygonPublisher_.publish(mapRegionPolygon_);
  return true;
}

unsigned int MapRegionVisualization::getNPoints()
{
  return 4;
}

} /* namespace */
