/*
 * SensorProcessor.cpp
 *
 *  Created on: Jun 18, 2014
 *      Author: hannes
 */

#include <elevation_mapping/SensorProcessor.hpp>

//PCL
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>

//TF
#include <tf_conversions/tf_eigen.h>

namespace elevation_mapping {

SensorProcessor::SensorProcessor(tf::TransformListener& transformListener):
				transformListener_(transformListener), mapFrameId_(""), baseFrameId_("")
{
	transformationSensorToMap_.setIdentity();
	transformListenerTimeout_.fromSec(1.0);
}

SensorProcessor::~SensorProcessor() {}

bool SensorProcessor::process(
		const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudInput,
		const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput,
		Eigen::VectorXf& variances)
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudClean(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*pointCloudInput, *pointCloudClean);
	cleanPointCloud(pointCloudClean);

	ros::Time timeStamp;
	timeStamp.fromNSec(1000.0 * pointCloudClean->header.stamp);

	if (!updateTransformations(pointCloudClean->header.frame_id, timeStamp)) return false;

	if (!transformPointCloud(pointCloudClean, pointCloudOutput, mapFrameId_)) return false;

	if (!computeVariances(pointCloudClean, robotPoseCovariance, variances)) return false;

	return true;
}

//Frame ID accessors
void SensorProcessor::setBaseFrameId(std::string baseFrameId)
{
	baseFrameId_ = baseFrameId;
}

void SensorProcessor::setMapFrameId(std::string mapFrameId)
{
	mapFrameId_ = mapFrameId;
}

std::string SensorProcessor::getBaseFrameId() const
{
	return baseFrameId_;
}

std::string SensorProcessor::getMapFrameId() const
{
	return mapFrameId_;
}

//Sensor parameter accessors
double SensorProcessor::getSensorParameter(std::size_t index) const
{
	return sensorParameters_.at(index);
}

std::string SensorProcessor::getSensorParameterName(std::size_t index) const
{
	return sensorParameterNames_.at(index);
}

void SensorProcessor::setTransformListenerTimeout(ros::Duration timeout)
{
	transformListenerTimeout_ = timeout;
}

ros::Duration SensorProcessor::getTransformListenerTimeout() const
{
	return transformListenerTimeout_;
}

/* Private */

bool SensorProcessor::updateTransformations(std::string sensorFrameId, ros::Time timeStamp)
{
	try
	{
		transformListener_.waitForTransform(sensorFrameId, mapFrameId_, timeStamp, ros::Duration(1.0));

		tf::StampedTransform transformTf;
		transformListener_.lookupTransform(mapFrameId_, sensorFrameId, timeStamp, transformTf);
		poseTFToEigen(transformTf, transformationSensorToMap_);

		transformListener_.lookupTransform(baseFrameId_, sensorFrameId, timeStamp, transformTf); // TODO Why wrong direction?
				Eigen::Affine3d transform;
		poseTFToEigen(transformTf, transform);
		rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
		translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();

		transformListener_.lookupTransform(mapFrameId_, baseFrameId_, timeStamp, transformTf); // TODO Why wrong direction?
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

bool SensorProcessor::transformPointCloud(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTransformed,
		const std::string& targetFrame)
{
	pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transformationSensorToMap_.cast<float>());
	pointCloudTransformed->header.frame_id = targetFrame;

	ROS_DEBUG("ElevationMap: Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
			ros::Time(pointCloudTransformed->header.stamp).toSec());
	return true;
}
} /* namespace elevation_mapping */


