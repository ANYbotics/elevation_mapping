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
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"
#include "elevation_map_visualization/visualizations/ElevationVisualization.hpp"
#include "elevation_map_visualization/visualizations/ElevationMeshVisualization.hpp"
#include "elevation_map_visualization/visualizations/MapRegionVisualization.hpp"
#include "elevation_map_visualization/visualizations/VarianceVisualization.hpp"

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
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("Elevation map visualization node started.");

  visualizations_.push_back(unique_ptr<VisualizationBase>(new ElevationVisualization(nodeHandle_)));
  visualizations_.push_back(unique_ptr<VisualizationBase>(new ElevationMeshVisualization(nodeHandle_)));
  visualizations_.push_back(unique_ptr<VisualizationBase>(new MapRegionVisualization(nodeHandle_)));
  visualizations_.push_back(unique_ptr<VisualizationBase>(new VarianceVisualization(nodeHandle_)));
  mapMarkerArrayMessage_.markers.resize(visualizations_.size());

  readParameters();

  mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &ElevationMapVisualization::elevationMapCallback, this);
  mapMarkerArrayPublisher_ = nodeHandle_.advertise<visualization_msgs::MarkerArray>("elevation_map_marker_array", 1, true);

  initialize();
}

ElevationMapVisualization::~ElevationMapVisualization()
{

}

bool ElevationMapVisualization::readParameters()
{
  nodeHandle_.param("elevation_map_topic", mapTopic_, string("/elevation_mapping/elevation_map"));

  for (auto& visualization : visualizations_)
  {
    if(!visualization->readParameters()) return false;
  }

  return true;
}

bool ElevationMapVisualization::initialize()
{
  for (unsigned int i = 0; i < visualizations_.size(); i++)
  {
    visualization_msgs::Marker& marker = mapMarkerArrayMessage_.markers.at(i);
    visualizations_.at(i)->initialize(&marker);
  }

  ROS_INFO("Elevation map visualization initialized.");
  return true;
}

void ElevationMapVisualization::elevationMapCallback(
    const elevation_map_msg::ElevationMap& map)
{
  if (mapMarkerArrayPublisher_.getNumSubscribers () < 1) return;

  ROS_DEBUG("ElevationVisualization received an elevation map (time stamp %f) for visualization.", map.header.stamp.toSec());

  for (auto& visualization : visualizations_)
  {
    if (!visualization->generateVisualization(map))
      ROS_ERROR("Generating visualization failed.");
  }

  mapMarkerArrayPublisher_.publish(mapMarkerArrayMessage_);
}

} /* namespace */
