/*
 * VisualizationBase.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_map_visualization/visualizations/VisualizationBase.hpp"

// Elevation Mapping
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"

namespace elevation_map_visualization {

VisualizationBase::VisualizationBase()
{
  marker_ = nullptr;
}

VisualizationBase::~VisualizationBase()
{

}

bool VisualizationBase::initialize(visualization_msgs::Marker* marker)
{
  if (marker == nullptr) return false;
  marker_ = marker;
  marker_->lifetime = ros::Duration();
  marker_->action = visualization_msgs::Marker::ADD;
  return true;
}

} /* namespace elevation_map_visualization */
