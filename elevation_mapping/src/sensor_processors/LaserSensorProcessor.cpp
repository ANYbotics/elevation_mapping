/*
 * KinectSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"

#include <pcl/filters/passthrough.h>
#include <limits>
#include <string>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

/*!
 * Anisotropic laser range sensor model:
 * standardDeviationInBeamDirection = minRadius
 * standardDeviationOfBeamRadius = beamConstant + beamAngle * measurementDistance
 *
 * Taken from: Pomerleau, F.; Breitenmoser, A; Ming Liu; Colas, F.; Siegwart, R.,
 * "Noise characterization of depth sensors for surface inspections,"
 * International Conference on Applied Robotics for the Power Industry (CARPI), 2012.
 */

LaserSensorProcessor::LaserSensorProcessor(ros::NodeHandle& nodeHandle, const SensorProcessorBase::GeneralParameters& generalParameters)
    : SensorProcessorBase(nodeHandle, generalParameters) {}

LaserSensorProcessor::~LaserSensorProcessor() = default;

bool LaserSensorProcessor::readParameters() {
  SensorProcessorBase::readParameters();
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  nodeHandle_.param("sensor_processor/min_radius", parameters.sensorParameters_["min_radius"], 0.0);
  nodeHandle_.param("sensor_processor/beam_angle", parameters.sensorParameters_["beam_angle"], 0.0);
  nodeHandle_.param("sensor_processor/beam_constant", parameters.sensorParameters_["beam_constant"], 0.0);
  return true;
}

bool LaserSensorProcessor::computeVariances(const PointCloudType::ConstPtr pointCloud,
                                            const Eigen::Matrix<double, 6, 6>& robotPoseCovariance, Eigen::VectorXf& variances) {
  const Parameters parameters{parameters_.getData()};
  variances.resize(pointCloud->size());

  // Projection vector (P).
  const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

  // Sensor Jacobian (J_s).
  const Eigen::RowVector3f sensorJacobian =
      projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

  // Robot rotation covariance matrix (Sigma_q).
  Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

  // Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
  const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
  const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
  const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
  const Eigen::Matrix3f B_r_BS_skew =
      kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

  const float varianceNormal = parameters.sensorParameters_.at("min_radius") * parameters.sensorParameters_.at("min_radius");
  const float beamConstant = parameters.sensorParameters_.at("beam_constant");
  const float beamAngle = parameters.sensorParameters_.at("beam_angle");
  for (size_t i = 0; i < pointCloud->size(); ++i) {
    // TODO(needs assignment): Move this loop body into a function for better unit testing.
    // For every point in point cloud.

    // Preparation.
    const auto& point = pointCloud->points[i];
    const Eigen::Vector3f pointVector(point.x, point.y, point.z);  // S_r_SP // NOLINT(cppcoreguidelines-pro-type-union-access)

    // Measurement distance.
    const float measurementDistance = pointVector.norm();

    // Compute sensor covariance matrix (Sigma_S) with sensor model.
    float varianceLateral = beamConstant + beamAngle * measurementDistance;
    varianceLateral *= varianceLateral;

    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    // Robot rotation Jacobian (J_q).
    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    const Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    // Measurement variance for map (error propagation law).
    float heightVariance = 0.0;  // sigma_p
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    // Copy to list.
    variances(i) = heightVariance;
  }

  return true;
}

}  // namespace elevation_mapping
