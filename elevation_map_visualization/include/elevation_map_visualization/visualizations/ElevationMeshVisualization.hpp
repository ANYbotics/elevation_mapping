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

  //! Height of the cube markers [m].
  double markerHeight_;

  //! Showing empty cell in visualization.
  bool showEmptyCells_;

  //! Base color for the map visualization.
  std_msgs::ColorRGBA baseColor_;

  //! Color for the empty cells of the map.
  std_msgs::ColorRGBA emptyCellColor_;

  //! Set colors from map color data.
  bool isSetColorFromMap_;

  //! Set colors from map variance data.
  bool isSetColorFromVariance_;

  //! Set colors from map elevation data.
  bool isSetColorFromHeight_;

  //! Set color saturation from map variance data.
  bool isSetSaturationFromVariance_;

  //! Set color alpha from map variance data.
  bool isSetAlphaFromVariance_;

  //! Colors for the lower and upper variance values.
  std_msgs::ColorRGBA lowerVarianceColor_;
  std_msgs::ColorRGBA upperVarianceColor_;

  //! Lower and upper value of the variance for mapping from
  //! the variance data to color properties.
  double varianceLowerValue_;
  double varianceUpperValue_;

  //! Lower and upper value of the elevation for mapping from
  //! the elevation data to color properties.
  double elevationLowerValue_;
  double elevationUpperValue_;

  //! Minimum and maximum saturation of the colors.
  double minMarkerSaturation_;
  double maxMarkerSaturation_;

  //! Minimum and maximum color alpha values.
  double minMarkerAlpha_;
  double maxMarkerAlpha_;
};

} /* namespace elevation_map_visualization */
