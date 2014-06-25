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
   * @param nodeHandle the ROS node handle.
   */
  VisualizationBase(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~VisualizationBase();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  virtual bool readParameters() = 0;

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

 protected:

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Pointer to the marker.
  // This could be made nicer with visualization_msgs::MarkerPtr,
  // however we'll need to figure out how to create a shared_pointer
  // to an existing object in the MarkerArray.
  visualization_msgs::Marker* marker_;
};

} /* namespace elevation_map_visualization */
