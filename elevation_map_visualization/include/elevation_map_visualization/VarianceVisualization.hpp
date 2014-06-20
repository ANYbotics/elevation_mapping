/*
 * VarianceVisualization.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Elevation Mapping
#include "elevation_map_visualization/VisualizationBase.hpp"
#include "elevation_map_msg/ElevationMap.h"
#include "elevation_map_visualization/ElevationMapVisualizationHelpers.hpp"

// ROS
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace elevation_map_visualization {

/*!
 * Visualization of the variances as sigma bounds. Sigma bounds are shown as dots (spheres)
 * above the cells.
 */
class VarianceVisualization : protected VisualizationBase
{
 public:

  /*!
   * Constructor.
   */
  VarianceVisualization();

  /*!
   * Destructor.
   */
  virtual ~VarianceVisualization();

  /*!
   * Initialization.
   * @param marker the pointer to the marker that is populated with visualization
   * information.
   */
  bool initialize(visualization_msgs::Marker* marker);

  /*!
   * Generates the visualization.
   * @param map the elevation map to visualize.
   * @return true if successful.
   */
  bool generateVisualization(const elevation_map_msg::ElevationMap& map);

 private:

  //! Sigma bound that is visualized.
  double sigmaBound_;
};

} /* namespace elevation_map_visualization */
