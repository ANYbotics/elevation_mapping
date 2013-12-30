/*
 * EigenConversions.cpp
 *
 *  Created on: Nov 26, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "EigenConversions.hpp"

// StarlETH Elevation Map
#include "ElevationMapHelpers.hpp"

// ROS
#include <ros/ros.h>

namespace starleth_elevation_msg {

bool matrixEigenToMultiArrayMessage(const Eigen::MatrixXd& e, std_msgs::Float64MultiArray& m)
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

bool multiArrayMessageToMatrixEigen(std_msgs::Float64MultiArray& m, Eigen::MatrixXd& e)
{
  if (e.IsRowMajor != starleth_elevation_msg::isRowMajor(m))
  {
    ROS_ERROR("starleth_elevation_msg: multiArrayMessageToMatrixEigen() failed because the storage order is not compatible.");
    return false;
  }

  e.resize(starleth_elevation_msg::getRows(m), starleth_elevation_msg::getCols(m));

  e = Eigen::Map<MatrixXd>(m.data.data(), starleth_elevation_msg::getRows(m), starleth_elevation_msg::getCols(m));

  return true;
}

} // namespace
