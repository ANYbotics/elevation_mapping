/*
 * RobotMotionMapUpdater.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_mapping/RobotMotionMapUpdater.hpp"

// Kindr
#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>
#include <kindr/linear_algebra/LinearAlgebra.hpp>

using namespace std;
using namespace Eigen;
using namespace kindr::rotations::eigen_impl;

namespace elevation_mapping {

RobotMotionMapUpdater::RobotMotionMapUpdater()
{
  previousRobotPoseCovariance_.setZero();
}

RobotMotionMapUpdater::~RobotMotionMapUpdater()
{

}

bool RobotMotionMapUpdater::update(
    ElevationMap& map, const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& robotPose,
    const Eigen::Matrix<double, 6, 6>& robotPoseCovariance, const ros::Time& time)
{
  // Check if update necessary.
  if ((robotPoseCovariance - previousRobotPoseCovariance_).all() == 0) return false;

  // Initialize update data.
  Array2i size = map.getRawGridMap().getBufferSize();
  MatrixXf varianceUpdate(size(0), size(1));
  MatrixXf horizontalVarianceUpdateX(size(0), size(1));
  MatrixXf horizontalVarianceUpdateY(size(0), size(1));

  // Covariance matrices.
  Matrix3d previousPositionCovariance = previousRobotPoseCovariance_.topLeftCorner<3, 3>();
  Matrix3d positionCovariance = robotPoseCovariance.topLeftCorner<3, 3>();
  Matrix3d previousRotationCovariance = previousRobotPoseCovariance_.bottomRightCorner<3, 3>();
  Matrix3d rotationCovariance = robotPoseCovariance.bottomRightCorner<3, 3>();

  // Parent to elevation map frame rotation (C_IM^T = C_SM^T * C_IS^T)
  Matrix3d parentToMapRotation = RotationMatrixPD(map.getPose().getRotation()).matrix().transpose();

  // Translation Jacobian (J_r)
  Matrix3d translationJacobian = -parentToMapRotation;

  // Translation variance update (for all points the same).
  Vector3f translationVarianceUpdate = (translationJacobian *
                                        (positionCovariance - previousPositionCovariance) *
                                        translationJacobian.transpose()).diagonal().cast<float>();

  // Robot/sensor position (I_r_IS, for all points the same).
  kindr::phys_quant::eigen_impl::Position3D robotPosition = robotPose.getPosition();

  // For each cell in map.
  for (unsigned int i = 0; i < size(0); ++i)
  {
    for (unsigned int j = 0; j < size(1); ++j)
    {
      kindr::phys_quant::eigen_impl::Position3D cellPosition; // I_r_IP

      if (map.getPosition3dInRobotParentFrame(Array2i(i, j), cellPosition))
      {
        // Rotation Jacobian (J_q)
        Matrix3d rotationJacobian = parentToMapRotation
            * kindr::linear_algebra::getSkewMatrixFromVector((cellPosition - robotPosition).vector());

        // Rotation variance update.
        Vector3f rotationVarianceUpdate = (rotationJacobian *
                                           (rotationCovariance - previousRotationCovariance) *
                                           rotationJacobian.transpose()).diagonal().cast<float>();

        // Variance update.
        varianceUpdate(i, j) = translationVarianceUpdate.z() + rotationVarianceUpdate.z();
        horizontalVarianceUpdateX(i, j) = translationVarianceUpdate.x() + rotationVarianceUpdate.x();
        horizontalVarianceUpdateY(i, j) = translationVarianceUpdate.y() + rotationVarianceUpdate.y();
      }
      else
      {
        // Cell invalid.
        varianceUpdate(i, j) = numeric_limits<float>::infinity();
        horizontalVarianceUpdateX(i, j) = numeric_limits<float>::infinity();
        horizontalVarianceUpdateY(i, j) = numeric_limits<float>::infinity();
      }

    }
  }

  map.update(varianceUpdate, horizontalVarianceUpdateX, horizontalVarianceUpdateY, time);

  previousRobotPoseCovariance_ = robotPoseCovariance;

  return true;
}

} /* namespace */
