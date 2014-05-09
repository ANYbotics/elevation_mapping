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

namespace elevation_map_msg {

bool multiArrayMessageToMatrixEigen(std_msgs::Float32MultiArray& m, Eigen::MatrixXf& e)
{
  if (e.IsRowMajor != elevation_map_msg::isRowMajor(m))
  {
    ROS_ERROR("elevation_map_msg: multiArrayMessageToMatrixEigen() failed because the storage order is not compatible.");
    return false;
  }

  e.resize(elevation_map_msg::getRows(m), elevation_map_msg::getCols(m));

  e = Eigen::Map<MatrixXf>(m.data.data(), elevation_map_msg::getRows(m), elevation_map_msg::getCols(m));

  return true;
}


} // namespace
