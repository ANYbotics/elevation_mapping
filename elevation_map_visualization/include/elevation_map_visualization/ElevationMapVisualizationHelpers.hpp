/*
 * ElevationMapVisualizationHelpers.hpp
 *
 *  Created on: Jun 24, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen/Core>

namespace elevation_map_visualization {

void getColorMessageFromColorVector(std_msgs::ColorRGBA& colorMessage, const Eigen::Vector3f& colorVector);

void getColorVectorFromColorMessage(Eigen::Vector3f& colorVector, const std_msgs::ColorRGBA& colorMessage);

bool setColorFromMap(std_msgs::ColorRGBA& color, const unsigned long& colorValue);
bool setColorChannelFromVariance(float& color, const double variance, const double varianceLowerValue, const double varianceUpperValue, bool invert = false);

/*!
 * @note Based on "changeSaturation" function by Darel Rex Finley.
 * @param color
 * @param variance
 * @return
 */
bool setSaturationFromVariance(std_msgs::ColorRGBA& color, const double variance, const double varianceLowerValue,
                               const double varianceUpperValue, const double maxMarkerSaturation,
                               const double minMarkerSaturation);

bool setColorFromHeight(std_msgs::ColorRGBA& color, const double height, const double elevationLowerValue, const double elevationUpperValue);

double computeLinearMapping(
    const double& sourceValue, const double& sourceLowerValue, const double& sourceUpperValue,
    const double& mapLowerValue, const double& mapUpperValue);

} /* namespace */
