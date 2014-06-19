/*
 * AslamSensorProcessor.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Hannes Keller
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <elevation_mapping/AslamSensorProcessor.hpp>

#include <pcl/filters/passthrough.h>
#include <vector>

namespace elevation_mapping {

AslamSensorProcessor::AslamSensorProcessor(tf::TransformListener& transformListener):
				SensorProcessor(transformListener)
{
	sensorParameterNames_.resize(6);
	sensorParameters_.resize(6);

	sensorParameterNames_[0] = "sensor_cutoff_min_depth";
	sensorParameterNames_[1] = "sensor_cutoff_max_depth";
	sensorParameterNames_[2] = "sensor_model_normal_factor_a";
	sensorParameterNames_[3] = "sensor_model_normal_factor_b";
	sensorParameterNames_[4] = "sensor_model_normal_factor_c";
	sensorParameterNames_[5] = "sensor_model_lateral_factor";
}

AslamSensorProcessor::~AslamSensorProcessor() {}

/* Private */

bool AslamSensorProcessor::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter;
	pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;

	passThroughFilter.setInputCloud(pointCloud);
	passThroughFilter.setFilterFieldName("z");
	passThroughFilter.setFilterLimits(sensorParameters_[0], sensorParameters_[1]);
	// This makes the point cloud also dense (no NaN points).
	passThroughFilter.filter(tempPointCloud);
	tempPointCloud.is_dense = true;
	pointCloud->swap(tempPointCloud);

	ROS_DEBUG("ElevationMap: cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
	return true;
}

bool AslamSensorProcessor::computeVariances(
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

	// Preparations for#include <pcl/common/transforms.h> robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
	const Eigen::Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
	const Eigen::RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
	const Eigen::Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
	const Eigen::Matrix3f B_r_BS_skew = kindr::linear_algebra::getSkewMatrixFromVector(Eigen::Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

	for (unsigned int i = 0; i < pointCloud->size(); ++i)
	{
		// For every point in point cloud.

		// Preparation.
		auto& point = pointCloud->points[i];
		Eigen::Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
		float heightVariance = 0.0; // sigma_p

		// Measurement distance.
		float measurementDistance = pointVector.norm();

		// Compute sensor covariance matrix (Sigma_S) with sensor model.
		float varianceNormal =
				pow(sensorParameters_[2] + sensorParameters_[3] *
						pow(measurementDistance - sensorParameters_[3], 2), 2);
		float varianceLateral = pow(sensorParameters_[5] * measurementDistance, 2);
		Eigen::Matrix3f sensorVariance = Eigen::Matrix3f::Zero();
		sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

		// Robot rotation Jacobian (J_q).
		const Eigen::Matrix3f C_SB_transpose_times_S_r_SP_skew = kindr::linear_algebra::getSkewMatrixFromVector(Eigen::Vector3f(C_SB_transpose * pointVector));
		Eigen::RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

		// Measurement variance for map (error propagation law).
		heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
		heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

		// Copy to list.
		variances(i) = heightVariance;
	}

	return true;
}

} /* namespace elevation_mapping */




