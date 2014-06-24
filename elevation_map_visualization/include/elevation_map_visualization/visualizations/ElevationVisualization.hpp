/*
 * ElevationVisualization.hpp
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
 * Visualization of the elevation as cubes. Populates a visualization marker with the information
 * from an elevation map message.
 */
class ElevationVisualization
{
 public:

  /*!
   * Constructor.
   */
  ElevationVisualization();

  /*!
   * Destructor.
   */
  virtual ~ElevationVisualization();

  /*!
   * Initialization.
   * @param marker the pointer to the marker that is populated with visualization
   * information.
   */
  void initialize(visualization_msgs::Marker* marker);

  /*!
   * Generates the visualization.
   * @param map the elevation map to visualize.
   * @return true if successful.
   */
  bool generateVisualization(const elevation_map_msg::ElevationMap& map);

  friend class ElevationMapVisualization;

 private:

  //! Pointer to the marker.
  // TODO: This could be made nicer with visualization_msgs::MarkerPtr,
  // however we'll need to figure out how to create a shared_pointer
  // to an existing object in the MarkerArray.
  visualization_msgs::Marker* marker_;

  //! Sigma bound that is visualized.
  double sigmaBound_;
};

} /* namespace elevation_map_visualization */
