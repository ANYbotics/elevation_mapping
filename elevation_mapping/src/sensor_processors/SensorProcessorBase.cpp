/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

#include <kindr_ros/kindr_ros.hpp>
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <limits>
#include <math.h>
#include <vector>

namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(ros::NodeHandle& nodeHandle, tf2_ros::Buffer& tfBuffer)
    : nodeHandle_(nodeHandle),
      tfBuffer_(tfBuffer),
      ignorePointsUpperThreshold_(std::numeric_limits<double>::infinity()),
      ignorePointsLowerThreshold_(-std::numeric_limits<double>::infinity())
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  tfBuffer_.setUsingDedicatedThread(true);
	transformationSensorToMap_.setIdentity();
}

SensorProcessorBase::~SensorProcessorBase() {}

bool SensorProcessorBase::readParameters()
{
  nodeHandle_.param("sensor_frame_id", sensorFrameId_, std::string("/sensor")); // TODO Fail if parameters are not found.
  nodeHandle_.param("robot_base_frame_id", robotBaseFrameId_, std::string("/robot"));
  nodeHandle_.param("map_frame_id", mapFrameId_, std::string("/map"));
  nodeHandle_.param("sensor_processor/ignore_points_above", ignorePointsUpperThreshold_, std::numeric_limits<double>::infinity());
  nodeHandle_.param("sensor_processor/ignore_points_below", ignorePointsLowerThreshold_, -std::numeric_limits<double>::infinity());
  return true;
}

bool SensorProcessorBase::process(
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudInput,
		const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudMapFrame,
		Eigen::VectorXf& variances)
{
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloudInput->header.stamp);
  if (!updateTransformations(timeStamp)) return false;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudSensorFrame(new pcl::PointCloud<pcl::PointXYZRGB>);
	transformPointCloud(pointCloudInput, pointCloudSensorFrame, sensorFrameId_);
	cleanPointCloud(pointCloudSensorFrame);

	if (!transformPointCloud(pointCloudSensorFrame, pointCloudMapFrame, mapFrameId_)) return false;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds({pointCloudMapFrame, pointCloudSensorFrame});
  removePointsOutsideLimits(pointCloudMapFrame, pointClouds);
	if (!computeVariances(pointCloudSensorFrame, robotPoseCovariance, variances)) return false;

	return true;
}

bool SensorProcessorBase::updateTransformations(const ros::Time& timeStamp)
{
  try {
    std::string errorMessage;
    if (!tfBuffer_.canTransform(sensorFrameId_, mapFrameId_, timeStamp, &errorMessage)) {
      ROS_WARN("Could not lookup TF transform from %s to %s.", mapFrameId_.c_str(), sensorFrameId_.c_str());
      return false;
    }

    // Sensor to base.
    geometry_msgs::TransformStamped transformStamped;
    transformStamped = tfBuffer_.lookupTransform(robotBaseFrameId_, sensorFrameId_, timeStamp);
    Transform transform;
    kindr_ros::convertFromRosGeometryMsg(transformStamped.transform, transform);
    translationBaseToSensorInBaseFrame_ = transform.getPosition();
    rotationSensorToBase_ = transform.getRotation();

    // Base to map.
    transformStamped = tfBuffer_.lookupTransform(mapFrameId_, robotBaseFrameId_, timeStamp);
    kindr_ros::convertFromRosGeometryMsg(transformStamped.transform, transform);
    translationMapToBaseInMapFrame_ = transform.getPosition();
    rotationBaseToMap_ = transform.getRotation();

    // Map to sensor.
    transformStamped = tfBuffer_.lookupTransform(mapFrameId_, sensorFrameId_, timeStamp);
    kindr_ros::convertFromRosGeometryMsg(transformStamped.transform, transformationSensorToMap_);

  } catch (tf2::TransformException &errorMessage) {
    ROS_ERROR("%s", errorMessage.what());
    return false;
  }

  return true;
}

bool SensorProcessorBase::transformPointCloud(
		pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTransformed,
		const std::string& targetFrame)
{
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloud->header.stamp);
  const std::string inputFrameId(pointCloud->header.frame_id);

  geometry_msgs::TransformStamped transformStamped;
  try {
    std::string errorMessage;
    if (!tfBuffer_.canTransform(targetFrame, inputFrameId, timeStamp, &errorMessage)) {
      ROS_WARN("Could not lookup TF transform from %s to %s.", inputFrameId.c_str(), targetFrame.c_str());
      return false;
    }
    transformStamped = tfBuffer_.lookupTransform(targetFrame, inputFrameId, timeStamp);
  } catch (tf2::TransformException &errorMessage) {
    ROS_ERROR("%s", errorMessage.what());
    return false;
  }

  Transform transform;
  kindr_ros::convertFromRosGeometryMsg(transformStamped.transform, transform);
  pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.getTransformationMatrix().cast<float>());
  pointCloudTransformed->header.frame_id = targetFrame;

	ROS_DEBUG("Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
			ros::Time(pointCloudTransformed->header.stamp).toSec());
	return true;
}

void SensorProcessorBase::removePointsOutsideLimits(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr reference, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& pointClouds)
{
  if (!std::isfinite(ignorePointsLowerThreshold_) && !std::isfinite(ignorePointsUpperThreshold_)) return;
  ROS_DEBUG("Limiting point cloud to the height interval of [%f, %f] relative to the robot base.", ignorePointsLowerThreshold_, ignorePointsUpperThreshold_);

  pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z"); // TODO: Should this be configurable?
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsUpperThreshold_;
  passThroughFilter.setFilterLimits(relativeLowerThreshold, relativeUpperThreshold);
  pcl::IndicesPtr insideIndeces(new std::vector<int>);
  passThroughFilter.filter(*insideIndeces);

  for (auto& pointCloud : pointClouds) {
    pcl::ExtractIndices<pcl::PointXYZRGB> extractIndicesFilter;
    extractIndicesFilter.setInputCloud(pointCloud);
    extractIndicesFilter.setIndices(insideIndeces);
    pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;
    extractIndicesFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }

  ROS_DEBUG("removePointsOutsideLimits() reduced point cloud to %i points.", (int) pointClouds[0]->size());
}

} /* namespace elevation_mapping */

