/*
 * StereoSensorProcessor.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Hannes Keller
 */

#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"

// PCL
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>

// STD
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

StereoSensorProcessor::StereoSensorProcessor(ros::NodeHandle& nodeHandle, const SensorProcessorBase::GeneralParameters& generalParameters)
    : SensorProcessorBase(nodeHandle, generalParameters), originalWidth_(1) {}

StereoSensorProcessor::~StereoSensorProcessor() = default;

bool StereoSensorProcessor::readParameters() {
  SensorProcessorBase::readParameters();
  auto [parameters, parameterGuard]{parameters_.getDataToWrite()};
  nodeHandle_.param("sensor_processor/p_1", parameters.sensorParameters_["p_1"], 0.0);
  nodeHandle_.param("sensor_processor/p_2", parameters.sensorParameters_["p_2"], 0.0);
  nodeHandle_.param("sensor_processor/p_3", parameters.sensorParameters_["p_3"], 0.0);
  nodeHandle_.param("sensor_processor/p_4", parameters.sensorParameters_["p_4"], 0.0);
  nodeHandle_.param("sensor_processor/p_5", parameters.sensorParameters_["p_5"], 0.0);
  nodeHandle_.param("sensor_processor/lateral_factor", parameters.sensorParameters_["lateral_factor"], 0.0);
  nodeHandle_.param("sensor_processor/depth_to_disparity_factor", parameters.sensorParameters_["depth_to_disparity_factor"], 0.0);
  nodeHandle_.param("sensor_processor/cutoff_min_depth", parameters.sensorParameters_["cutoff_min_depth"],
                    std::numeric_limits<double>::min());
  nodeHandle_.param("sensor_processor/cutoff_max_depth", parameters.sensorParameters_["cutoff_max_depth"],
                    std::numeric_limits<double>::max());
  return true;
}

bool StereoSensorProcessor::computeVariances(const PointCloudType::ConstPtr pointCloud,
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

  // Preparations for#include <pcl/common/transforms.h> robot rotation Jacobian (J_q) to minimize computation for every point in point
  // cloud.
  const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
  const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
  const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
  const Eigen::Matrix3f B_r_BS_skew =
      kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    // For every point in point cloud.

    // Preparation.
    pcl::PointXYZRGBConfidenceRatio point = pointCloud->points[i];
    double disparity =
        parameters.sensorParameters_.at("depth_to_disparity_factor") / point.z;  // NOLINT(cppcoreguidelines-pro-type-union-access)
    Eigen::Vector3f pointVector(point.x, point.y, point.z);  // S_r_SP // NOLINT(cppcoreguidelines-pro-type-union-access)
    float heightVariance = 0.0;                              // sigma_p

    // Measurement distance.
    float measurementDistance = pointVector.norm();

    // Compute sensor covariance matrix (Sigma_S) with sensor model.
    float varianceNormal =
        pow(parameters.sensorParameters_.at("depth_to_disparity_factor") / pow(disparity, 2), 2) *
        ((parameters.sensorParameters_.at("p_5") * disparity + parameters.sensorParameters_.at("p_2")) *
             sqrt(pow(parameters.sensorParameters_.at("p_3") * disparity + parameters.sensorParameters_.at("p_4") - getJ(i), 2) +
                  pow(240 - getI(i), 2)) +
         parameters.sensorParameters_.at("p_1"));
    float varianceLateral = pow(parameters.sensorParameters_.at("lateral_factor") * measurementDistance, 2);
    Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    // Robot rotation Jacobian (J_q).
    const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
    Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    // Measurement variance for map (error propagation law).
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    // Copy to list.
    variances(i) = heightVariance;
  }

  return true;
}

bool StereoSensorProcessor::filterPointCloudSensorType(const PointCloudType::Ptr pointCloud) {
  const Parameters parameters{parameters_.getData()};
  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter;
  PointCloudType tempPointCloud;

  // cutoff points with z values
  passThroughFilter.setInputCloud(pointCloud);
  passThroughFilter.setFilterFieldName("z");
  passThroughFilter.setFilterLimits(parameters.sensorParameters_.at("cutoff_min_depth"),
                                    parameters.sensorParameters_.at("cutoff_max_depth"));
  passThroughFilter.filter(tempPointCloud);
  pointCloud->swap(tempPointCloud);

  return true;
}

int StereoSensorProcessor::getI(int index) {
  // TODO(max): Figure out originalWidth_ value.
  return indices_[index] / originalWidth_;
}

int StereoSensorProcessor::getJ(int index) {
  return indices_[index] % originalWidth_;
}

}  // namespace elevation_mapping
