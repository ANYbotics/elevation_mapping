/*
 * CovarianceMapUpdater.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include "ElevationMap.hpp"

// Eigen
#include <Eigen/Core>

// Kindr
#include <kindr/poses/eigen/HomogeneousTransformation.hpp>

namespace elevation_mapping {

/*
 * Computes the map variance update from the pose covariance of the robot.
 */
class CovarianceMapUpdater
{
 public:
  CovarianceMapUpdater();
  virtual ~CovarianceMapUpdater();

  /*!
   * Computes the model update for the elevation map based on the pose covariance and
   * adds the update to the map.
   * @param[in] map the elevation map to be updated.
   * @param[in] robotPose the latest pose.
   * @param[in] robotPoseCovariance the latest pose covariance matrix.
   * @return true if successful.
   */
  bool update(ElevationMap& map,
              const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& robotPose,
              const Eigen::Matrix<double, 6, 6>& robotPoseCovariance);

 private:
  Eigen::Matrix<double, 6, 6> previousRobotPoseCovariance_;
};

} /* namespace */
