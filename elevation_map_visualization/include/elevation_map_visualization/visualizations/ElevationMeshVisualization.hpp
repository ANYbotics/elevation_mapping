/*
 * ElevationMeshVisualization.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Elevation Mapping
#include "elevation_map_visualization/visualizations/VisualizationBase.hpp"

// ROS
#include "std_msgs/ColorRGBA.h"

namespace elevation_map_visualization {

/*!
 * Visualization of the elevation as cubes.
 */
class ElevationMeshVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ElevationMeshVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationMeshVisualization();

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

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

  /*!
   * Set the color for a cell depending on the map information.
   * @param color the color to be set.
   * @param elevation the cell elevation data.
   * @param variance the cell variance data.
   * @param colorValue the cell color data.
   * @param isEmtpyCell if the cell is an empty cell.
   */
  void setColor(std_msgs::ColorRGBA& color, const double elevation, const double variance, const unsigned long colorValue, bool isEmtpyCell);

  //! Base color for the map visualization.
  std_msgs::ColorRGBA baseColor_;
};

} /* namespace elevation_map_visualization */
