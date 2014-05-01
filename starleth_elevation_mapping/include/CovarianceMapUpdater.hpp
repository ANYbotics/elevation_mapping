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

namespace starleth_elevation_mapping {

/*
 * Computes the map variance update from the pose covariance of the robot.
 */
class CovarianceMapUpdater
{
 public:
  CovarianceMapUpdater();
  virtual ~CovarianceMapUpdater();

  /*!
   * Computes and returns the update based on the pose covariance.
   * @param[in] robotPose the latest pose.
   * @param[in] robotPoseCovariance the latest pose covariance matrix.
   * @param[out] varianceUpdate the update of the variance.
   * @param[out] horizontalVarianceUpdateX the update of the horizonal variances in x-direction.
   * @param[out] horizontalVarianceUpdateY the update of the horizonal variances in y-direction.
   * @return true if successful.
   */
  bool update(ElevationMap& map,
              const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& robotPose,
              const Eigen::Matrix<double, 6, 6>& robotPoseCovariance);

 private:
  Eigen::Matrix<double, 6, 6> previousRobotPoseCovariance_;
};

} /* namespace starleth_elevation_mapping */
