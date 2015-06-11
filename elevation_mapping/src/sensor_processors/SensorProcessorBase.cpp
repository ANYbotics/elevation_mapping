/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

//PCL
#include <pcl/pcl_base.h>
#include <pcl/common/transforms.h>
#include <pcl/common/io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

//TF
#include <tf_conversions/tf_eigen.h>

// STL
#include <limits>
#include <math.h>
#include <vector>

namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(ros::NodeHandle& nodeHandle, tf::TransformListener& transformListener)
    : nodeHandle_(nodeHandle),
      transformListener_(transformListener),
      mapFrameId_(""),
      robotBaseFrameId_("")
{
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
	transformationSensorToMap_.setIdentity();
	transformListenerTimeout_.fromSec(1.0);
}

SensorProcessorBase::~SensorProcessorBase() {}

bool SensorProcessorBase::readParameters()
{
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
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudSensorFrame(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*pointCloudInput, *pointCloudSensorFrame);
	cleanPointCloud(pointCloudSensorFrame);
	ros::Time timeStamp;
	timeStamp.fromNSec(1000 * pointCloudSensorFrame->header.stamp);

	if (!updateTransformations(pointCloudSensorFrame->header.frame_id, timeStamp)) return false;
	if (!transformPointCloud(pointCloudSensorFrame, pointCloudMapFrame, mapFrameId_)) return false;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pointClouds({pointCloudMapFrame, pointCloudSensorFrame});
  removePointsOutsideLimits(pointCloudMapFrame, pointClouds);
	if (!computeVariances(pointCloudSensorFrame, robotPoseCovariance, variances)) return false;
	return true;
}

bool SensorProcessorBase::updateTransformations(const std::string& sensorFrameId,
                                                const ros::Time& timeStamp)
{
  try {
    transformListener_.waitForTransform(sensorFrameId, mapFrameId_, timeStamp, ros::Duration(1.0));

    tf::StampedTransform transformTf;
    transformListener_.lookupTransform(mapFrameId_, sensorFrameId, timeStamp, transformTf);
    poseTFToEigen(transformTf, transformationSensorToMap_);

    transformListener_.lookupTransform(robotBaseFrameId_, sensorFrameId, timeStamp, transformTf);  // TODO Why wrong direction?
    Eigen::Affine3d transform;
    poseTFToEigen(transformTf, transform);
    rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
    translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();

    transformListener_.lookupTransform(mapFrameId_, robotBaseFrameId_, timeStamp, transformTf);  // TODO Why wrong direction?
    poseTFToEigen(transformTf, transform);
    rotationMapToBase_.setMatrix(transform.rotation().matrix());
    translationMapToBaseInMapFrame_.toImplementation() = transform.translation();

    return true;
  } catch (tf::TransformException &ex) {
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

void SensorProcessorBase::removePointsOutsideLimits(
    pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr reference, std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& pointClouds)
{
  if (!std::isfinite(ignorePointsLowerThreshold_) && !std::isfinite(ignorePointsUpperThreshold_)) return;
  ROS_DEBUG("Limiting point cloud to the height interval of [%f, %f] relative to the robot base.", ignorePointsLowerThreshold_, ignorePointsUpperThreshold_);

  pcl::PassThrough<pcl::PointXYZRGB> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z"); // Should this be configurable?
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

