/*
 * RobotMotionMapUpdater.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/poses/eigen/HomogeneousTransformation.hpp>

// ROS (time)
#include <ros/ros.h>

namespace elevation_mapping {

/*!
 * Computes the map variance update from the pose covariance of the robot.
 */
class RobotMotionMapUpdater
{
 public:

  /*!
   * Constructor.
   */
  RobotMotionMapUpdater(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~RobotMotionMapUpdater();

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Computes the model update for the elevation map based on the pose covariance and
   * adds the update to the map.
   * @param[in] map the elevation map to be updated.
   * @param[in] robotPose the latest pose.
   * @param[in] robotPoseCovariance the latest pose covariance matrix.
   * @param[in] time the time of the update.
   * @return true if successful.
   */
  bool update(ElevationMap& map,
              const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& robotPose,
              const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
              const ros::Time& time);

 private:

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Robot pose covariance from the previous update.
  Eigen::Matrix<double, 6, 6> previousRobotPoseCovariance_;

  //! Scaling factor for the covariance matrix (default all ones).
  Eigen::Array<double, 6, 6> covarianceScale_;
};

} /* namespace */
