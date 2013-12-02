/*
 * ElevationMapTransformations.hpp
 *
 *  Created on: Nov 27, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <starleth_elevation_msg/ElevationMap.h>

// Eigen
#include <Eigen/Core>

// ROS
#include <std_msgs/Float64MultiArray.h>

// STL
#include <map>

using namespace Eigen;

namespace starleth_elevation_msg {

enum class StorageIndices {
    Column,
    Row
};

extern std::map<StorageIndices, std::string> storageIndexNames;

bool isRowMajor(const std_msgs::Float64MultiArray& messageData);

unsigned int getCols(const std_msgs::Float64MultiArray& messageData);

unsigned int getRows(const std_msgs::Float64MultiArray& messageData);

void getDistanceOfFirstCellFromCenter(Eigen::Vector2d& distance, const starleth_elevation_msg::ElevationMap& map);

Matrix2i getIndexToPositionDirectionTransformation();

/*!
 * Returns the 1d array index to access the map data based on the 2d matrix indeces
 * @param index the 2d matrix indeces
 * @param map the reference to the map
 * @return the 1d array index
 */
unsigned int get1dIndexFrom2dIndex(
    const Eigen::Vector2i& index,
    const starleth_elevation_msg::ElevationMap& map);

/*!
 * Returns the 2d matrix indeces based on the 1d array index (NOT TESTED!)
 * @param n the 1d array index
 * @param map the reference to the map
 * @return 2d matrix indeces
 */
Eigen::Vector2i get2dIndexFrom1dIndex(
    unsigned int n, const starleth_elevation_msg::ElevationMap& map);

/*!
 * Gets the position of the center of the cell in the map coordinate system
 * from the index of the cell
 * @param [out] position the center of the cell in the map coordinate system (2d)
 * @param [in] index of the cell
 * @param [in] map the reference to the map
 * @return true
 */
bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Vector2i& index,
                          const starleth_elevation_msg::ElevationMap& map);

} // namespace
