/*
 * RobotMotionMapUpdater.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */
#include "elevation_mapping/RobotMotionMapUpdater.hpp"

// Kindr
#include <kindr/Core>

// using namespace kindr;

namespace elevation_mapping {

RobotMotionMapUpdater::RobotMotionMapUpdater(ros::NodeHandle nodeHandle) : nodeHandle_(nodeHandle), covarianceScale_(1.0) {
  previousReducedCovariance_.setZero();
  previousUpdateTime_ = ros::Time::now();
  // TODO(max): How to initialize previousRobotPose_?
}

RobotMotionMapUpdater::~RobotMotionMapUpdater() = default;

bool RobotMotionMapUpdater::readParameters() {
  nodeHandle_.param("robot_motion_map_update/covariance_scale", covarianceScale_, 1.0);
  return true;
}

bool RobotMotionMapUpdater::update(ElevationMap& map, const Pose& robotPose, const PoseCovariance& robotPoseCovariance,
                                   const ros::Time& time) {
  const PoseCovariance robotPoseCovarianceScaled = covarianceScale_ * robotPoseCovariance;

  // Check if update necessary.
  if (previousUpdateTime_ == time) {
    return false;
  }

  // Initialize update data.
  grid_map::Size size = map.getRawGridMap().getSize();
  grid_map::Matrix varianceUpdate(size(0), size(1));  // TODO(max): Make as grid map?
  grid_map::Matrix horizontalVarianceUpdateX(size(0), size(1));
  grid_map::Matrix horizontalVarianceUpdateY(size(0), size(1));
  grid_map::Matrix horizontalVarianceUpdateXY(size(0), size(1));

  // Relative convariance matrix between two robot poses.
  ReducedCovariance reducedCovariance;
  computeReducedCovariance(robotPose, robotPoseCovarianceScaled, reducedCovariance);
  ReducedCovariance relativeCovariance;
  computeRelativeCovariance(robotPose, reducedCovariance, relativeCovariance);

  // Retrieve covariances for (24).
  Covariance positionCovariance = relativeCovariance.topLeftCorner<3, 3>();
  Covariance rotationCovariance(Covariance::Zero());
  rotationCovariance(2, 2) = relativeCovariance(3, 3);

  // Map to robot pose rotation (R_B_M = R_I_B^T * R_I_M).
  kindr::RotationMatrixPD mapToRobotRotation = kindr::RotationMatrixPD(robotPose.getRotation().inverted() * map.getPose().getRotation());
  kindr::RotationMatrixPD mapToPreviousRobotRotationInverted =
      kindr::RotationMatrixPD(previousRobotPose_.getRotation().inverted() * map.getPose().getRotation()).inverted();

  // Translation Jacobian (J_r) (25).
  Eigen::Matrix3d translationJacobian = -mapToRobotRotation.matrix().transpose();

  // Translation variance update (for all points the same).
  Eigen::Vector3f translationVarianceUpdate =
      (translationJacobian * positionCovariance * translationJacobian.transpose()).diagonal().cast<float>();

  // Map-robot relative position (M_r_Bk_M, for all points the same).
  // Preparation for (25): M_r_BP = R_I_M^T (I_r_I_M - I_r_I_B) + M_r_M_P
  // R_I_M^T (I_r_I_M - I_r_I_B):
  const kindr::Position3D positionRobotToMap =
      map.getPose().getRotation().inverseRotate(map.getPose().getPosition() - previousRobotPose_.getPosition());

  auto& heightLayer = map.getRawGridMap()["elevation"];

  // For each cell in map. // TODO(max): Change to new iterator.
  for (unsigned int i = 0; i < static_cast<unsigned int>(size(0)); ++i) {
    for (unsigned int j = 0; j < static_cast<unsigned int>(size(1)); ++j) {
      kindr::Position3D cellPosition;  // M_r_MP

      const auto height = heightLayer(i, j);
      if (std::isfinite(height)) {
        grid_map::Position position;
        map.getRawGridMap().getPosition({i, j}, position);
        cellPosition = {position.x(), position.y(), height};

        // Rotation Jacobian J_R (25)
        const Eigen::Matrix3d rotationJacobian =
            -kindr::getSkewMatrixFromVector((positionRobotToMap + cellPosition).vector()) * mapToPreviousRobotRotationInverted.matrix();

        // Rotation variance update.
        const Eigen::Matrix2f rotationVarianceUpdate =
            (rotationJacobian * rotationCovariance * rotationJacobian.transpose()).topLeftCorner<2, 2>().cast<float>();

        // Variance update.
        varianceUpdate(i, j) = translationVarianceUpdate.z();
        horizontalVarianceUpdateX(i, j) = translationVarianceUpdate.x() + rotationVarianceUpdate(0, 0);
        horizontalVarianceUpdateY(i, j) = translationVarianceUpdate.y() + rotationVarianceUpdate(1, 1);
        horizontalVarianceUpdateXY(i, j) = rotationVarianceUpdate(0, 1);
      } else {
        // Cell invalid. // TODO(max): Change to new functions
        varianceUpdate(i, j) = std::numeric_limits<float>::infinity();
        horizontalVarianceUpdateX(i, j) = std::numeric_limits<float>::infinity();
        horizontalVarianceUpdateY(i, j) = std::numeric_limits<float>::infinity();
        horizontalVarianceUpdateXY(i, j) = std::numeric_limits<float>::infinity();
      }
    }
  }

  map.update(varianceUpdate, horizontalVarianceUpdateX, horizontalVarianceUpdateY, horizontalVarianceUpdateXY, time);
  previousReducedCovariance_ = reducedCovariance;
  previousRobotPose_ = robotPose;
  return true;
}

bool RobotMotionMapUpdater::computeReducedCovariance(const Pose& robotPose, const PoseCovariance& robotPoseCovariance,
                                                     ReducedCovariance& reducedCovariance) {
  // Get augmented Jacobian (A.4).
  kindr::EulerAnglesZyxPD eulerAngles(robotPose.getRotation());
  double tanOfPitch = tan(eulerAngles.pitch());
  // (A.5)
  Eigen::Matrix<double, 1, 3> yawJacobian(cos(eulerAngles.yaw()) * tanOfPitch, sin(eulerAngles.yaw()) * tanOfPitch, 1.0);
  Eigen::Matrix<double, 4, 6> jacobian;
  jacobian.setZero();
  jacobian.topLeftCorner(3, 3).setIdentity();
  jacobian.bottomRightCorner(1, 3) = yawJacobian;

  // (A.3)
  reducedCovariance = jacobian * robotPoseCovariance * jacobian.transpose();
  return true;
}

bool RobotMotionMapUpdater::computeRelativeCovariance(const Pose& robotPose, const ReducedCovariance& reducedCovariance,
                                                      ReducedCovariance& relativeCovariance) {
  // Rotation matrix of z-align frame R_I_tilde_B.
  const kindr::RotationVectorPD rotationVector_I_B(robotPose.getRotation());
  const kindr::RotationVectorPD rotationVector_I_tilde_B(0.0, 0.0, rotationVector_I_B.vector().z());
  const kindr::RotationMatrixPD R_I_tilde_B(rotationVector_I_tilde_B);

  // Compute translational velocity from finite differences.
  kindr::Position3D positionInRobotFrame =
      previousRobotPose_.getRotation().inverseRotate(robotPose.getPosition() - previousRobotPose_.getPosition());
  kindr::Velocity3D v_Delta_t(positionInRobotFrame);  // (A.8)

  // Jacobian F (A.8).
  Jacobian F;
  F.setIdentity();
  // TODO(max): Why does Eigen::Vector3d::UnitZ() not work?
  F.topRightCorner(3, 1) = kindr::getSkewMatrixFromVector(Eigen::Vector3d(0.0, 0.0, 1.0)) * R_I_tilde_B.matrix() * v_Delta_t.vector();

  // Jacobian inv(G) * Delta t (A.14).
  Jacobian inv_G_Delta_t;
  inv_G_Delta_t.setZero();
  inv_G_Delta_t(3, 3) = 1.0;
  Jacobian inv_G_transpose_Delta_t(inv_G_Delta_t);
  inv_G_Delta_t.topLeftCorner(3, 3) = R_I_tilde_B.matrix().transpose();
  inv_G_transpose_Delta_t.topLeftCorner(3, 3) = R_I_tilde_B.matrix();

  // Relative (reduced) robot covariance (A.13).
  relativeCovariance = inv_G_Delta_t * (reducedCovariance - F * previousReducedCovariance_ * F.transpose()) * inv_G_transpose_Delta_t;

  return true;
}

}  // namespace elevation_mapping
