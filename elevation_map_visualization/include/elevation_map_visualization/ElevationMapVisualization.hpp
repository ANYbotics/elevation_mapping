/*
 * ElevationMapVisualization.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// Elevation Mapping
#include "elevation_map_visualization/visualizations/VisualizationBase.hpp"
#include "elevation_map_visualization/visualizations/MapRegionVisualization.hpp"
#include "elevation_map_msg/ElevationMap.h"

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

// Eigen
#include <Eigen/Core>

// Unique Pointer
#include <memory>

namespace elevation_map_visualization {

/*!
 * Visualized an elevation map by publishing marker arrays that can be viewed in rviz.
 */
class ElevationMapVisualization
{
 public:

  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ElevationMapVisualization(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationMapVisualization();

  /*!
   * Callback function for the elevation map.
   * @param map the elevation map to be visualized.
   */
  void elevationMapCallback(const elevation_map_msg::ElevationMap& map);

 private:

  /*!
   * Read parameters from ROS.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Initialization.
   * @return true if successful.
   */
  bool initialize();

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! ROS subscriber to the elevation map.
  ros::Subscriber mapSubscriber_;

  //! ROS publisher of the marker array.
  ros::Publisher mapMarkerArrayPublisher_;

  //! Nnumber of visualization layers/modules.
  const unsigned int nVisualizations_;

  //! List of visualization modules, can be separately activated in rviz.
  std::vector<std::unique_ptr<VisualizationBase>> visualizations_;

  //! Marker array message which is published to ROS.
  visualization_msgs::MarkerArray mapMarkerArrayMessage_;

  //! Topic name of the elevation map to be visualized.
  std::string mapTopic_;

  // TODO Add to new general framework.
  MapRegionVisualization mapRegionVisualization_;
};

} /* elevation_map_visualization */
