/*
 * EigenConversions.hpp
 *
 *  Created on: Nov 26, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// StarlETH Elevation Map
#include "ElevationMessageHelpers.hpp"

// ROS
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/UInt32MultiArray.h>

// Eigen
#include <Eigen/Core>

namespace starleth_elevation_msg {

/*!
 * Converts an Eigen matrix into a ROS MultiArray message.
 * Both column- and row-major matrices are allowed, and the type
 * will be marked in the layout labels.
 *
 * Note: This function copies the data, because std::vector<> is not able
 * to map external memory.
 *
 * @param [in] e the Eigen matrix to be converted.
 * @param [out] m the ROS message to which the data will be converted.
 * @return true if successful
 */
template<typename EigenType_, typename MessageType_>
bool matrixEigenToMultiArrayMessage(const EigenType_& e, MessageType_& m)
{
  m.layout.dim.resize(nDimensions());
  m.layout.dim[0].stride = e.size();
  m.layout.dim[0].size = e.outerSize();
  m.layout.dim[1].stride = e.innerSize();
  m.layout.dim[1].size = e.innerSize();

  if(e.IsRowMajor)
  {
    m.layout.dim[0].label = storageIndexNames[StorageIndices::Row];
    m.layout.dim[1].label = storageIndexNames[StorageIndices::Column];
  }
  else
  {
    m.layout.dim[0].label = storageIndexNames[StorageIndices::Column];
    m.layout.dim[1].label = storageIndexNames[StorageIndices::Row];
  }

  m.data.insert(m.data.begin() + m.layout.data_offset, e.data(), e.data() + e.size());

  return true;
}


/*!
 * Converts a ROS Float64MultiArray message into an Eigen matrix.
 * Both column- and row-major message types are allowed.
 *
 * TODO: Check if this actually works.
 *
 * @param [in] m the ROS message to be converted.
 * @param [in] e the Eigen matrix to which the data will be converted.
 * @return true if successful
 */
bool multiArrayMessageToMatrixEigen(std_msgs::Float32MultiArray& m, Eigen::MatrixXf& e);

} // namespace
