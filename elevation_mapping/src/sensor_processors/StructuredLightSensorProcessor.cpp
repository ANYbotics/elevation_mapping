/*
 * StructuredLightSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp>

// PCL
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

// STD
#include <vector>
#include <limits>
#include <string>

namespace elevation_mapping {

/*! StructuredLight-type (structured light) sensor model:
 * standardDeviationInNormalDirection = sensorModelNormalFactorA_ + sensorModelNormalFactorB_ * (measurementDistance - sensorModelNormalFactorC_)^2;
 * standardDeviationInLateralDirection = sensorModelLateralFactor_ * measurementDistance
 * Taken from: Nguyen, C. V., Izadi, S., & Lovell, D., Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking, 2012.
 */

StructuredLightSensorProcessor::StructuredLightSensorProcessor(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : SensorProcessorBase(nodeHandle, transformListener)
{

}

StructuredLightSensorProcessor::~StructuredLightSensorProcessor()
{

}

bool StructuredLightSensorProcessor::readParameters()
{
  SensorProcessorBase::readParameters();
  nodeHandle_.param("sensor_processor/normal_factor_a", sensorParameters_["normal_factor_a"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_b", sensorParameters_["normal_factor_b"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_c", sensorParameters_["normal_factor_c"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_d", sensorParameters_["normal_factor_d"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_e", sensorParameters_["normal_factor_e"], 0.0);
  nodeHandle_.param("sensor_processor/lateral_factor", sensorParameters_["lateral_factor"], 0.0);
  nodeHandle_.param("sensor_processor/cutoff_min_depth", sensorParameters_["cutoff_min_depth"], std::numeric_limits<double>::min());
  nodeHandle_.param("sensor_processor/cutoff_max_depth", sensorParameters_["cutoff_max_depth"], std::numeric_limits<double>::max());
  return true;
}

bool StructuredLightSensorProcessor::computeVariances(
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
		const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
		Eigen::VectorXf& variances)
{
	variances.resize(pointCloud->size());

	// Projection vector (P).
	const Eigen::RowVector3f projectionVector = Eigen::RowVector3f::UnitZ();

	// Sensor Jacobian (J_s).
	const Eigen::RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

	// Robot rotation covariance matrix (Sigma_q).
	Eigen::Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

	// Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
	const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
	const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
	const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
	const Eigen::Matrix3f B_r_BS_skew = kindr::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
		// For every point in point cloud.

		// Preparation.
		auto& point = pointCloud->points[i];
		Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
		float heightVariance = 0.0; // sigma_p

		// Measurement distance.
		float measurementDistance = pointVector.z();

		// Compute sensor covariance matrix (Sigma_S) with sensor model.
                float deviationNormal = sensorParameters_.at("normal_factor_a")
                    + sensorParameters_.at("normal_factor_b")
                        * (measurementDistance - sensorParameters_.at("normal_factor_c")) * (measurementDistance - sensorParameters_.at("normal_factor_c"))
                    + sensorParameters_.at("normal_factor_d") * pow(measurementDistance, sensorParameters_.at("normal_factor_e"));
		float varianceNormal = deviationNormal * deviationNormal;
		float deviationLateral = sensorParameters_.at("lateral_factor") * measurementDistance;
		float varianceLateral = deviationLateral * deviationLateral;
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

bool StructuredLightSensorProcessor::filterPointCloudSensorType(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud) {
    pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter;
    pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;

    // cutoff points with z values
    passThroughFilter.setInputCloud(pointCloud);
    passThroughFilter.setFilterFieldName("z");
    passThroughFilter.setFilterLimits(sensorParameters_.at("cutoff_min_depth"),
                                      sensorParameters_.at("cutoff_max_depth"));
    passThroughFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);

    return true;
}

} /* namespace */
