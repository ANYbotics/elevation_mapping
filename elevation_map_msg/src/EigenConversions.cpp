/*
 * EigenConversions.cpp
 *
 *  Created on: Nov 26, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "EigenConversions.hpp"

// ROS
#include <ros/ros.h>

namespace starleth_elevation_msg {

bool multiArrayMessageToMatrixEigen(std_msgs::Float32MultiArray& m, Eigen::MatrixXf& e)
{
  if (e.IsRowMajor != starleth_elevation_msg::isRowMajor(m))
  {
    ROS_ERROR("starleth_elevation_msg: multiArrayMessageToMatrixEigen() failed because the storage order is not compatible.");
    return false;
  }

  e.resize(starleth_elevation_msg::getRows(m), starleth_elevation_msg::getCols(m));

  e = Eigen::Map<MatrixXf>(m.data.data(), starleth_elevation_msg::getRows(m), starleth_elevation_msg::getCols(m));

  return true;
}


} // namespace
