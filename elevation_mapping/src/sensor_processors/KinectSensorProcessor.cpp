/*
 * KinectSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <elevation_mapping/sensor_processors/KinectSensorProcessor.hpp>

#include <pcl/filters/passthrough.h>
#include <vector>
#include <limits>
#include <string>

namespace elevation_mapping {

/*! Kinect-type (structured light) sensor model:
 * standardDeviationInNormalDirection = sensorModelNormalFactorA_ + sensorModelNormalFactorB_ * (measurementDistance - sensorModelNormalFactorC_)^2;
 * standardDeviationInLateralDirection = sensorModelLateralFactor_ * measurementDistance
 * Taken from: Nguyen, C. V., Izadi, S., & Lovell, D., Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking, 2012.
 */

KinectSensorProcessor::KinectSensorProcessor(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : SensorProcessorBase(nodeHandle, transformListener)
{

}

KinectSensorProcessor::~KinectSensorProcessor()
{

}

bool KinectSensorProcessor::readParameters()
{
  SensorProcessorBase::readParameters();
  nodeHandle_.param("sensor_processor/cutoff_min_depth", sensorParameters_["cutoff_min_depth"], std::numeric_limits<double>::min());
  nodeHandle_.param("sensor_processor/cutoff_max_depth", sensorParameters_["cutoff_max_depth"], std::numeric_limits<double>::max());
  nodeHandle_.param("sensor_processor/normal_factor_a", sensorParameters_["normal_factor_a"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_b", sensorParameters_["normal_factor_b"], 0.0);
  nodeHandle_.param("sensor_processor/normal_factor_c", sensorParameters_["normal_factor_c"], 0.0);
  nodeHandle_.param("sensor_processor/lateral_factor", sensorParameters_["lateral_factor"], 0.0);
  nodeHandle_.param("robot_base_frame_id", robotBaseFrameId_, std::string("/robot"));
  nodeHandle_.param("map_frame_id", mapFrameId_, std::string("/map"));

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  transformListenerTimeout_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!transformListenerTimeout_.isZero());

  return true;
}

bool KinectSensorProcessor::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
	pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter;
	pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;

	passThroughFilter.setInputCloud(pointCloud);
	passThroughFilter.setFilterFieldName("z");
	passThroughFilter.setFilterLimits(sensorParameters_.at("cutoff_min_depth"), sensorParameters_.at("cutoff_max_depth"));
	// This makes the point cloud also dense (no NaN points).
	passThroughFilter.filter(tempPointCloud);
	tempPointCloud.is_dense = true;
	pointCloud->swap(tempPointCloud);

	ROS_DEBUG("cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
	return true;
}

bool KinectSensorProcessor::computeVariances(
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
				pow(sensorParameters_.at("normal_factor_a") + sensorParameters_.at("normal_factor_b") *
						pow(measurementDistance - sensorParameters_.at("normal_factor_c"), 2), 2);
		float varianceLateral = pow(sensorParameters_.at("lateral_factor") * measurementDistance, 2);
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

} /* namespace */
