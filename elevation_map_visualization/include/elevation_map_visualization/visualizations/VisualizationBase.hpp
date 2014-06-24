/*
 * VisualizationBase.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Elevation Mapping
#include "elevation_map_msg/ElevationMap.h"
#include "elevation_map_visualization/ElevationMapVisualizationHelpers.hpp"

namespace elevation_map_visualization {

/*!
 * Base class for visualizations of elevation maps. Populates a visualization marker
 * with the information from an elevation map message.
 */
class VisualizationBase
{
 public:

  /*!
   * Constructor.
   */
  VisualizationBase();

  /*!
   * Destructor.
   */
  virtual ~VisualizationBase();

  /*!
   * Initialization.
   * @param marker the pointer to the marker that is populated with visualization
   * information.
   * @return true if successful.
   */
  virtual bool initialize(visualization_msgs::Marker* marker);

  /*!
   * Generates the visualization.
   * @param map the elevation map to visualize.
   * @return true if successful.
   */
  virtual bool generateVisualization(const elevation_map_msg::ElevationMap& map) = 0;

  enum class MarkerTypes
  {
    Elevation,
    Variance,
    SurfaceNormal,
    Count
  };

 protected:

  //! Pointer to the marker.
  // TODO: This could be made nicer with visualization_msgs::MarkerPtr,
  // however we'll need to figure out how to create a shared_pointer
  // to an existing object in the MarkerArray.
  visualization_msgs::Marker* marker_;
};

} /* namespace elevation_map_visualization */
