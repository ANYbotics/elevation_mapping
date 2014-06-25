/*
 * ElevationMapMsgHelpers.hpp
 *
 *  Created on: Nov 27, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Elevation Map
#include "elevation_map_msg/ElevationMap.h"

// Eigen
#include <Eigen/Core>

// ROS
#include <std_msgs/Float32MultiArray.h>

// STL
#include <map>

using namespace Eigen;

namespace elevation_map_msg {

/*!
 * Returns the number of dimensions of the elevation map.
 * @return the number of dimensions.
 */
const int nDimensions();

enum class StorageIndices {
    Column,
    Row
};

//! Holds the names of the storage indeces.
extern std::map<StorageIndices, std::string> storageIndexNames;

/*!
 * Checks if message data is stored in row-major format.
 * @param[in] messageData the message data.
 * @return true if is in row-major format, false if is in column-major format.
 */
bool isRowMajor(const std_msgs::Float32MultiArray& messageData);

/*!
 * Returns the number of columns of the message data.
 * @param[in] messageData the message data.
 * @return the number of columns.
 */
unsigned int getCols(const std_msgs::Float32MultiArray& messageData);

/*!
 * Returns the number of rows of the message data.
 * @param[in] messageData the message data.
 * @return the number of rows.
 */
unsigned int getRows(const std_msgs::Float32MultiArray& messageData);

/*!
 * Checks if the cell holds valid data.
 * @param elevation the elevation of the cell.
 * @param variance the variance of the cell.
 * @return true if valid cell, false if empty or invalid.
 */
bool isValidCell(const float elevation, const float variance);

/*!
 * Returns the 1d array index to access the map data based on the 2d matrix indeces.
 * @param index the 2d matrix indeces.
 * @param map the reference to the map.
 * @return the 1d array index.
 */
unsigned int get1dIndexFrom2dIndex(
    const Eigen::Array2i& index,
    const elevation_map_msg::ElevationMap& map);

/*!
 * Returns the 2d matrix indeces based on the 1d array index (NOT TESTED!).
 * @param n the 1d array index.
 * @param map the reference to the map.
 * @return 2d matrix indeces.
 */
//Eigen::Array2i get2dIndexFrom1dIndex(
//    unsigned int n, const elevation_map_msg::ElevationMap& map);

/*!
 * Gets the position of the center of the cell in the map coordinate system
 * from the index of the cell.
 * @param [out] position the center of the cell in the map coordinate system (2d).
 * @param [in] index of the cell.
 * @param [in] map the reference to the map.
 * @return true if successful.
 */
bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Array2i& index,
                          const elevation_map_msg::ElevationMap& map);

/*!
 * Gets the index of the cell which contains a position in the map frame.
 * @param [out] index of the cell.
 * @param [in] position the position in the map frame.
 * @param [in] map the reference to the map.
 * @return true if successful, false if position outside of map.
 */
bool getIndexfromPosition(Eigen::Array2i& index,
                          const Eigen::Vector2d& position,
                          const elevation_map_msg::ElevationMap& map);

/*!
 * Gets the color vector (RGB from 0-255) from a color value (concatenated RGB values).
 * @param [in] colorValue the concatenated RGB color value.
 * @param [out] colorVector the color vector in RGB from 0-255.
 * @return true if successful.
 */
bool copyColorValueToVector(const unsigned long& colorValue, Eigen::Vector3i& colorVector);

/*!
 * Gets the color vector (RGB from 0.0-1.0) from a color value (concatenated RGB values).
 * @param [in] colorValue the concatenated RGB color value.
 * @param [out] colorVector the color vector in RGB from 0.0-1.0.
 * @return true if successful.
 */
bool copyColorValueToVector(const unsigned long& colorValue, Eigen::Vector3f& colorVector);

/*!
 * Gets the concatenated RGB color value from a color vector (RGB from 0-255).
 * @param [in] colorVector the color vector in RGB from 0-255.
 * @param [out] colorValue the concatenated RGB color value.
 * @return true if successful.
 */
bool copyColorVectorToValue(const Eigen::Vector3i& colorVector, unsigned long& colorValue);

} // namespace
