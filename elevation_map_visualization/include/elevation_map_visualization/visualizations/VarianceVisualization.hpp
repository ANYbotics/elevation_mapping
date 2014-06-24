/*
 * VarianceVisualization.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Elevation Mapping
#include "elevation_map_visualization/visualizations/VisualizationBase.hpp"

namespace elevation_map_visualization {

/*!
 * Visualization of the variances as sigma bounds. Sigma bounds are shown as dots (spheres)
 * above the cells.
 */
class VarianceVisualization : public VisualizationBase
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  VarianceVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~VarianceVisualization();

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
   * Set the color for a variance marker depending on the cell variance.
   * @param color the color of the marker to be set.
   * @param variance the cell variance data.
   */
  void setColor(std_msgs::ColorRGBA& color, const double variance);


  //! Sigma bound that is visualized.
  double sigmaBound_;

  //! Size (diameter of the spheres) of the markers [m].
  double markerSize_;

  //! Base color for the variance visualization.
  std_msgs::ColorRGBA baseColor_;

  //! Set colors from map variance data.
  bool isSetColorFromVariance_;

  //! Colors for the lower and upper variance values.
  std_msgs::ColorRGBA lowerVarianceColor_;
  std_msgs::ColorRGBA upperVarianceColor_;

  //! Lower and upper value of the variance for mapping from
  //! the variance to color properties.
  double varianceLowerValue_;
  double varianceUpperValue_;

};

} /* namespace elevation_map_visualization */
