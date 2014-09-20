/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

//PCL
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>

//TF
#include <tf_conversions/tf_eigen.h>

// STD
#include <limits>
#include <math.h>

namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : nodeHandle_(nodeHandle),
      transformListener_(transformListener),
      mapFrameId_(""),
      robotBaseFrameId_("")
{
	transformationSensorToMap_.setIdentity();
	transformListenerTimeout_.fromSec(1.0);
}

SensorProcessorBase::~SensorProcessorBase() {}

bool SensorProcessorBase::readParameters()
{
  nodeHandle_.param("sensor_processor/ignore_points_above", ignorePointsAbove_, std::numeric_limits<double>::max());
  nodeHandle_.param("sensor_processor/ignore_points_below", ignorePointsBelow_, std::numeric_limits<double>::min());
  return true;
}

bool SensorProcessorBase::process(
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudInput,
		const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput,
		Eigen::VectorXf& variances)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudClean(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*pointCloudInput, *pointCloudClean);
	cleanPointCloud(pointCloudClean);

	ros::Time timeStamp;
	timeStamp.fromNSec(1000 * pointCloudClean->header.stamp);

	if (!updateTransformations(pointCloudClean->header.frame_id, timeStamp)) return false;

	if (!transformPointCloud(pointCloudClean, pointCloudOutput, mapFrameId_)) return false;
	removePointsOutsideLimits(pointCloudOutput);

	if (!computeVariances(pointCloudClean, robotPoseCovariance, variances)) return false;

	return true;
}

bool SensorProcessorBase::updateTransformations(const std::string& sensorFrameId, const ros::Time& timeStamp)
{
	try
	{
		transformListener_.waitForTransform(sensorFrameId, mapFrameId_, timeStamp, ros::Duration(1.0));

		tf::StampedTransform transformTf;
		transformListener_.lookupTransform(mapFrameId_, sensorFrameId, timeStamp, transformTf);
		poseTFToEigen(transformTf, transformationSensorToMap_);

		transformListener_.lookupTransform(robotBaseFrameId_, sensorFrameId, timeStamp, transformTf); // TODO Why wrong direction?
		Eigen::Affine3d transform;
		poseTFToEigen(transformTf, transform);
		rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
		translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();

		transformListener_.lookupTransform(mapFrameId_, robotBaseFrameId_, timeStamp, transformTf); // TODO Why wrong direction?
		poseTFToEigen(transformTf, transform);
		rotationMapToBase_.setMatrix(transform.rotation().matrix());
		translationMapToBaseInMapFrame_.toImplementation() = transform.translation();

		return true;
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s", ex.what());
		return false;
	}
}

bool SensorProcessorBase::transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTransformed,
		const std::string& targetFrame)
{
	pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transformationSensorToMap_.cast<float>());
	pointCloudTransformed->header.frame_id = targetFrame;

	ROS_DEBUG("Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
			ros::Time(pointCloudTransformed->header.stamp).toSec());
	return true;
}

void SensorProcessorBase::removePointsOutsideLimits(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
  if (!std::isfinite(ignorePointsBelow_) && !std::isfinite(ignorePointsAbove_)) return;
  pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter;
  pcl::PointCloud<pcl::PointXYZRGB> tempPointCloud;
  passThroughFilter.setInputCloud(pointCloud);
  passThroughFilter.setFilterFieldName("z");
  passThroughFilter.setFilterLimits(-1000.0, 1.0); // TODO !!!
  passThroughFilter.filter(tempPointCloud);
  pointCloud->swap(tempPointCloud);
  ROS_DEBUG("removePointsOutsideLimits() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
}

} /* namespace elevation_mapping */

