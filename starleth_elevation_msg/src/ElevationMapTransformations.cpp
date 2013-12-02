/*
 * ElevationMapTransformations.cpp
 *
 *  Created on: Nov 27, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "ElevationMapTransformations.hpp"

// ROS
#include <ros/ros.h>

// Boost
#include <boost/assign.hpp>

namespace starleth_elevation_msg {

std::map<StorageIndices, std::string> storageIndexNames = boost::assign::map_list_of
    (StorageIndices::Column,  "column_index")
    (StorageIndices::Row, "row_index");

bool isRowMajor(const std_msgs::Float64MultiArray& messageData)
{
  if (messageData.layout.dim[0].label == storageIndexNames[StorageIndices::Column]) return false;
  else if (messageData.layout.dim[0].label == storageIndexNames[StorageIndices::Row]) return true;

  ROS_ERROR("starleth_elevation_msg: isRowMajor() failed because layout label is not set correctly.");
  return false;
}

unsigned int getCols(const std_msgs::Float64MultiArray& messageData)
{
  if (isRowMajor(messageData)) return messageData.layout.dim.at(1).size;
  return messageData.layout.dim.at(0).size;
}

unsigned int getRows(const std_msgs::Float64MultiArray& messageData)
{
  if (isRowMajor(messageData)) return messageData.layout.dim.at(0).size;
  return messageData.layout.dim.at(1).size;
}

void getDistanceOfFirstCellFromCenter(Eigen::Vector2d& distance, const starleth_elevation_msg::ElevationMap& map)
{
  // Distance of center of cell
  distance.x() = 0.5 * map.lengthInX - 0.5 * map.resolution;
  distance.y() = 0.5 * map.lengthInY - 0.5 * map.resolution;
}

Matrix2i getIndexToPositionDirectionTransformation()
{
  return -Matrix2i::Identity();
}

unsigned int get1dIndexFrom2dIndex(
    const Eigen::Vector2i& index,
    const starleth_elevation_msg::ElevationMap& map)
{
  unsigned int n;

  if(!isRowMajor(map.elevation)) n = map.elevation.layout.data_offset + index(1) * map.elevation.layout.dim[1].stride + index(0);
  else n = map.elevation.layout.data_offset + index(0) * map.elevation.layout.dim[1].stride + index(1);

  return n;
}

Eigen::Vector2i get2dIndexFrom1dIndex(
    unsigned int n, const starleth_elevation_msg::ElevationMap& map)
{
  Eigen::Vector2i index;
  index(1) = n - map.elevation.layout.data_offset % map.elevation.layout.dim[1].stride;
  index(0) = (int)((n - map.elevation.layout.data_offset - index(1)) / map.elevation.layout.dim[1].stride);
  return index;
}

bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Vector2i& index,
                          const starleth_elevation_msg::ElevationMap& map)
{
  Vector2d offset;
  getDistanceOfFirstCellFromCenter(offset, map);
  position = offset + map.resolution *
            (getIndexToPositionDirectionTransformation() * index).cast<double>();
  return true;
}

} // namespace
