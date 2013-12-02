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

/*!
 * Converts an Eigen matrix into a ROS Float64MultiArray message.
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
bool matrixEigenToMultiArrayMessage(const Eigen::MatrixXd& e, std_msgs::Float64MultiArray& m);

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
bool multiArrayMessageToMatrixEigen(std_msgs::Float64MultiArray& m, Eigen::MatrixXd& e);

} // namespace
