/*
 * ElevationVisualization.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// StarlETH Navigation
#include "MapRegionVisualization.hpp"
#include <starleth_elevation_msg/ElevationMap.h>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Core>

namespace starleth_elevation_visualization {

/*
 *
 */
class ElevationVisualization
{
 public:
  ElevationVisualization(ros::NodeHandle& nodeHandle);

  virtual ~ElevationVisualization();

  void elevationMapCallback(const starleth_elevation_msg::ElevationMap& map);

 private:
  bool readParameters();
  bool initializeVisualization();
  bool generateVisualization(const starleth_elevation_msg::ElevationMap& map);
  bool setColor(std_msgs::ColorRGBA& color, const double& elevation, const double& variance, const unsigned long& colorValue, bool isEmtpyCell);
  bool setColorFromMap(std_msgs::ColorRGBA& color, const unsigned long& colorValue);
  bool setColorChannelFromVariance(float& color, const double& variance, bool invert = false);

  /*!
   * @note Based on "changeSaturation" function by Darel Rex Finley.
   * @param color
   * @param variance
   * @return
   */
  bool setSaturationFromVariance(std_msgs::ColorRGBA& color, const double& variance);

  bool setColorFromHeight(std_msgs::ColorRGBA& color, const double& height);

  // TODO: Move this somewhere to a math library.
  double computeLinearMapping(
      const double& sourceValue, const double& sourceLowerValue, const double& sourceUpperValue,
      const double& mapLowerValue, const double& mapUpperValue);

  enum class MarkerTypes
  {
    ElevationMap,
    Count
  };

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber mapSubscriber_;
  ros::Publisher mapMarkerArrayPublisher_;
  visualization_msgs::MarkerArray mapMarkerArrayMessage_;

  MapRegionVisualization mapRegionVisualization_;

  // TODO Add struct for parameters
  std::string mapTopic_;
  double markerHeight_;
  bool isSetColorFromMap_;
  bool isSetColorFromVariance_;
  bool isSetColorFromHeight_;
  bool isSetSaturationFromVariance_;
  bool isSetAlphaFromVariance_;
  bool showEmptyCells_;
  double varianceLowerValue_;
  double varianceUpperValue_;
  double elevationLowerValue_;
  double elevationUpperValue_;
  double minMarkerSaturation_;
  double maxMarkerSaturation_;
  double minMarkerAlpha_;
  double maxMarkerAlpha_;
};

} /* namespace starleth_elevation_visualization */
