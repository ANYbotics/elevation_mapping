/*
 * CovarianceMapUpdater.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "CovarianceMapUpdater.hpp"

// STD
#include <iostream>

using namespace std;
using namespace Eigen;

namespace starleth_elevation_mapping {

CovarianceMapUpdater::CovarianceMapUpdater()
{
  previousRobotPoseCovariance_.setZero();
}

CovarianceMapUpdater::~CovarianceMapUpdater()
{

}

bool CovarianceMapUpdater::computeUpdate(
    const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
    Eigen::MatrixXf& varianceUpdate,
    Eigen::MatrixXf& horizontalVarianceUpdateX,
    Eigen::MatrixXf& horizontalVarianceUpdateY)
{
  varianceUpdate.setZero();
  horizontalVarianceUpdateX.setZero();
  horizontalVarianceUpdateY.setZero();

  Vector3d previousPositionVariance = previousRobotPoseCovariance_.diagonal().head(3);
  Vector3d positionVariance = robotPoseCovariance.diagonal().head(3);

  Vector3f variancePrediction = (positionVariance - previousPositionVariance).cast<float>();

  // Generate map update data.
  varianceUpdate = (varianceUpdate.array() + variancePrediction.z()).matrix();

  horizontalVarianceUpdateX = (horizontalVarianceUpdateX.array() + variancePrediction.x()).matrix();
  horizontalVarianceUpdateY = (horizontalVarianceUpdateY.array() + variancePrediction.y()).matrix();

  previousRobotPoseCovariance_ = robotPoseCovariance;

  return true;
}

} /* namespace starleth_elevation_mapping */
