/*
 * EigenConversions.hpp
 *
 *  Created on: Nov 26, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <std_msgs/Float64MultiArray.h>

// Eigen
#include <Eigen/Core>

namespace starleth_elevation_msg {

////! Converts an Eigen matrix into a Float64MultiArray message
//template <class Derived>
//void matrixEigenToMsg(const Eigen::MatrixBase<Derived>& e, std_msgs::Float64MultiArray& m)
//{
//  if (m.layout.dim.size() != 2)
//    m.layout.dim.resize(2);
//  m.layout.dim[0].stride = e.rows() * e.cols();
//  m.layout.dim[0].size = e.rows();
//  m.layout.dim[1].stride = e.cols();
//  m.layout.dim[1].size = e.cols();
//  if ((int)m.data.size() != e.size())
//    m.data.resize(e.size());
//  int ii = 0;
//  for (int i = 0; i < e.rows(); ++i)
//    for (int j = 0; j < e.cols(); ++j)
//      m.data[ii++] = e.coeff(i, j);
//}

} // namespace
