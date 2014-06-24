/*
 * ElevationMapVisualizationHelpers.cpp
 *
 *  Created on: Jun 24, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_map_visualization/ElevationMapVisualizationHelpers.hpp"

// Elevation Mapping
#include "elevation_map_msg/ElevationMap.h"
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"

using namespace Eigen;

namespace elevation_map_visualization {

void getColorMessageFromColorVector(std_msgs::ColorRGBA& colorMessage, const Eigen::Vector3f& colorVector)
{
  colorMessage.r = colorVector(0);
  colorMessage.g = colorVector(1);
  colorMessage.b = colorVector(2);
}

void getColorVectorFromColorMessage(Eigen::Vector3f& colorVector, const std_msgs::ColorRGBA& colorMessage)
{
  colorVector << colorMessage.r, colorMessage.g, colorMessage.b;
}

bool setColorFromMap(std_msgs::ColorRGBA& color, const unsigned long& colorValue)
{
  Vector3f colorVector;
  elevation_map_msg::copyColorValueToVector(colorValue, colorVector);
  getColorMessageFromColorVector(color, colorVector);
  return true;
}

bool setColorChannelFromVariance(float& colorChannel, const double variance, const double varianceLowerValue, const double varianceUpperValue, bool invert)
{
  double lowestVarianceValue = 0.0;
  double highestVarianceValue = 1.0;

  if (invert)
  {
    double tempValue = lowestVarianceValue;
    lowestVarianceValue = highestVarianceValue;
    highestVarianceValue = tempValue;
  }

  colorChannel = static_cast<float>(computeLinearMapping(variance, varianceLowerValue, varianceUpperValue, lowestVarianceValue, highestVarianceValue));

  return true;
}

bool setSaturationFromVariance(std_msgs::ColorRGBA& color, const double variance, const double varianceLowerValue,
                               const double varianceUpperValue, const double maxMarkerSaturation,
                               const double minMarkerSaturation)
{
  const Eigen::Array3f HspFactors(.299, .587, .114); // see http://alienryderflex.com/hsp.html
  float saturationChange = static_cast<float>(computeLinearMapping(variance, varianceLowerValue, varianceUpperValue, maxMarkerSaturation, minMarkerSaturation));
  Vector3f colorVector;
  getColorVectorFromColorMessage(colorVector, color);
  float perceivedBrightness = sqrt((colorVector.array().square() * HspFactors).sum());
  colorVector = perceivedBrightness + saturationChange * (colorVector.array() - perceivedBrightness);
  colorVector = (colorVector.array().min(Array3f::Ones())).matrix();
  getColorMessageFromColorVector(color, colorVector);
  return true;
}

bool setColorFromHeight(std_msgs::ColorRGBA& color, const double height, const double elevationLowerValue, const double elevationUpperValue)
{
  Vector3f hsl; // Hue: [0, 2 Pi], Saturation and Lightness: [0, 1]
  Vector3f rgb;

  hsl[0] = static_cast<float>(computeLinearMapping(height, elevationLowerValue, elevationUpperValue, 0.0, 2.0 * M_PI));
  hsl[1] = 1.0;
  hsl[2] = 1.0;

  float offset = 2.0 / 3.0 * M_PI;
  Array3f rgbOffset(0, -offset, offset);
  rgb = ((rgbOffset + hsl[0]).cos() + 0.5).min(Array3f::Ones()).max(Array3f::Zero()) * hsl[2];
  float white = Vector3f(0.3, 0.59, 0.11).transpose() * rgb;
  float saturation = 1.0 - hsl[1];
  rgb = rgb + ((-rgb.array() + white) * saturation).matrix();

  getColorMessageFromColorVector(color, rgb);
  return true;
}

double computeLinearMapping(
    const double& sourceValue, const double& sourceLowerValue, const double& sourceUpperValue,
    const double& mapLowerValue, const double& mapUpperValue)
{
  double m = (mapLowerValue - mapUpperValue) / (sourceLowerValue - sourceUpperValue);
  double b = mapUpperValue - m * sourceUpperValue;
  double mapValue = m * sourceValue + b;
  if (mapLowerValue < mapUpperValue)
  {
    mapValue = std::max(mapValue, mapLowerValue);
    mapValue = std::min(mapValue, mapUpperValue);
  }
  else
  {
    mapValue = std::min(mapValue, mapLowerValue);
    mapValue = std::max(mapValue, mapUpperValue);
  }
  return mapValue;
}

} /* namespace elevation_map_visualization */
