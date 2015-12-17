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
using namespace grid_map;
using namespace kindr::rotations::eigen_impl;

namespace elevation_mapping {

RobotMotionMapUpdater::RobotMotionMapUpdater(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle)
{
  previousRobotPoseCovariance_.setZero();
  covarianceScale_.setOnes();
}

RobotMotionMapUpdater::~RobotMotionMapUpdater()
{

}

bool RobotMotionMapUpdater::readParameters()
{
  nodeHandle_.param("robot_motion_map_update/covariance_scale_translation_x", covarianceScale_(0, 0), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_translation_y", covarianceScale_(1, 1), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_translation_z", covarianceScale_(2, 2), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_rotation_x", covarianceScale_(3, 3), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_rotation_y", covarianceScale_(4, 4), 1.0);
  nodeHandle_.param("robot_motion_map_update/covariance_scale_rotation_z", covarianceScale_(5, 5), 1.0);
  return true;
}

bool RobotMotionMapUpdater::update(
    ElevationMap& map, const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& robotPose,
    const Eigen::Matrix<double, 6, 6>& robotPoseCovariance, const ros::Time& time)
{
  Eigen::Matrix<double, 6, 6> robotPoseCovarianceScaled = (covarianceScale_ * robotPoseCovariance.array()).matrix();

  // Check if update necessary.
  if (((robotPoseCovarianceScaled - previousRobotPoseCovariance_).array() == 0.0).all()) return false;

  // Initialize update data.
  Size size = map.getRawGridMap().getSize();
  Matrix varianceUpdate(size(0), size(1));
  Matrix horizontalVarianceUpdateX(size(0), size(1));
  Matrix horizontalVarianceUpdateY(size(0), size(1));
  Matrix horizontalVarianceUpdateXY(size(0), size(1));

  // Covariance matrices.
  Eigen::Matrix3d previousPositionCovariance = previousRobotPoseCovariance_.topLeftCorner<3, 3>();
  Eigen::Matrix3d positionCovariance = robotPoseCovarianceScaled.topLeftCorner<3, 3>();
  Eigen::Matrix3d previousRotationCovariance = previousRobotPoseCovariance_.bottomRightCorner<3, 3>();
  Eigen::Matrix3d rotationCovariance = robotPoseCovarianceScaled.bottomRightCorner<3, 3>();

  // Parent to elevation map frame rotation (C_IM^T = C_SM^T * C_IS^T)
  Eigen::Matrix3d parentToMapRotation = RotationMatrixPD(map.getPose().getRotation()).matrix().transpose();

  // Translation Jacobian (J_r)
  Eigen::Matrix3d translationJacobian = -parentToMapRotation;

  // Translation variance update (for all points the same).
  Eigen::Vector3f translationVarianceUpdate = (translationJacobian *
                                        (positionCovariance - previousPositionCovariance) *
                                        translationJacobian.transpose()).diagonal().cast<float>();

  // Robot/sensor position (I_r_IS, for all points the same).
  kindr::phys_quant::eigen_impl::Position3D robotPosition = robotPose.getPosition();

  // For each cell in map. // TODO Change to new iterator.
  for (unsigned int i = 0; i < size(0); ++i)
  {
    for (unsigned int j = 0; j < size(1); ++j)
    {
      kindr::phys_quant::eigen_impl::Position3D cellPosition; // I_r_IP

      if (map.getPosition3dInRobotParentFrame(Index(i, j), cellPosition))
      {
        // Rotation Jacobian (J_q)
        Eigen::Matrix3d rotationJacobian = parentToMapRotation
            * kindr::linear_algebra::getSkewMatrixFromVector((cellPosition - robotPosition).vector());

        // Rotation variance update.
        Eigen::Matrix2f rotationVarianceUpdate = (rotationJacobian * (rotationCovariance - previousRotationCovariance) *
                                                 rotationJacobian.transpose()).topLeftCorner<2,2>().cast<float>();

        // Variance update.
        varianceUpdate(i, j) = translationVarianceUpdate.z();
        horizontalVarianceUpdateX(i, j) = translationVarianceUpdate.x() + rotationVarianceUpdate(0, 0);
        horizontalVarianceUpdateY(i, j) = translationVarianceUpdate.y() + rotationVarianceUpdate(1, 1);
        horizontalVarianceUpdateXY(i, j) = rotationVarianceUpdate(0, 1);
      }
      else
      {
        // Cell invalid. // TODO Change to new functions
        varianceUpdate(i, j) = numeric_limits<float>::infinity();
        horizontalVarianceUpdateX(i, j) = numeric_limits<float>::infinity();
        horizontalVarianceUpdateY(i, j) = numeric_limits<float>::infinity();
        horizontalVarianceUpdateXY(i, j) = numeric_limits<float>::infinity();
      }
    }
  }

  map.update(varianceUpdate, horizontalVarianceUpdateX, horizontalVarianceUpdateY, horizontalVarianceUpdateXY, time);
  previousRobotPoseCovariance_ = robotPoseCovarianceScaled;
  return true;
}

} /* namespace */
