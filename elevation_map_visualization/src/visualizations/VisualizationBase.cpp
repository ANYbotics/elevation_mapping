/*
 * VisualizationBase.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_map_visualization/visualizations/VisualizationBase.hpp"

namespace elevation_map_visualization {

VisualizationBase::VisualizationBase(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
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
  return true;
}

} /* namespace elevation_map_visualization */
