/*
 * MapRegionVisualization.hpp
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
 * Visualization of the region of the elevation map as border line.
 */
class MapRegionVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  MapRegionVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~MapRegionVisualization();

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

  //! Number of vertices of the map region visualization.
  const unsigned int nVertices_;

  //! Color of the map region visualization.
  std_msgs::ColorRGBA color_;

  //! Line width of the map region marker [m].
  double lineWidth_;

};

} /* namespace elevation_map_visualization */
