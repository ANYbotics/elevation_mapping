/*
 * SensorProcessorBase.cpp
 *
 *  Created on: Jun 6, 2014
 *      Author: Péter Fankhauser, Hannes Keller
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

// ROS
#include <geometry_msgs/TransformStamped.h>

// PCL
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>

// TF
#include <tf2_eigen/tf2_eigen.h>

// STL
#include <cmath>
#include <limits>
#include <vector>

#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"

namespace elevation_mapping {

SensorProcessorBase::SensorProcessorBase(ros::NodeHandle& nodeHandle, const GeneralParameters& generalConfig)
    : nodeHandle_(nodeHandle),
      transformListener_(transformBuffer_),
      ignorePointsUpperThreshold_(std::numeric_limits<double>::infinity()),
      ignorePointsLowerThreshold_(-std::numeric_limits<double>::infinity()),
      applyVoxelGridFilter_(false),
      firstTfAvailable_(false) {
  pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
  transformationSensorToMap_.setIdentity();
  generalParameters_ = generalConfig;
  ROS_DEBUG(
      "Sensor processor general parameters are:"
      "\n\t- robot_base_frame_id: %s"
      "\n\t- map_frame_id: %s",
      generalConfig.robotBaseFrameId_.c_str(), generalConfig.mapFrameId_.c_str());
}

SensorProcessorBase::~SensorProcessorBase() = default;

bool SensorProcessorBase::readParameters() {
  nodeHandle_.param("sensor_processor/ignore_points_above", ignorePointsUpperThreshold_, std::numeric_limits<double>::infinity());
  nodeHandle_.param("sensor_processor/ignore_points_below", ignorePointsLowerThreshold_, -std::numeric_limits<double>::infinity());

  nodeHandle_.param("sensor_processor/apply_voxelgrid_filter", applyVoxelGridFilter_, false);
  nodeHandle_.param("sensor_processor/voxelgrid_filter_size", sensorParameters_["voxelgrid_filter_size"], 0.0);
  return true;
}

bool SensorProcessorBase::process(const PointCloudType::ConstPtr pointCloudInput, const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
                                  const PointCloudType::Ptr pointCloudMapFrame, Eigen::VectorXf& variances, std::string sensorFrame) {
  sensorFrameId_ = sensorFrame;
  ROS_DEBUG("Sensor Processor processing for frame %s", sensorFrameId_.c_str());

  // Update transformation at timestamp of pointcloud
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloudInput->header.stamp);
  if (!updateTransformations(timeStamp)) {
    return false;
  }

  // Transform into sensor frame.
  PointCloudType::Ptr pointCloudSensorFrame(new PointCloudType);
  transformPointCloud(pointCloudInput, pointCloudSensorFrame, sensorFrameId_);

  // Remove Nans (optional voxel grid filter)
  filterPointCloud(pointCloudSensorFrame);

  // Specific filtering per sensor type
  filterPointCloudSensorType(pointCloudSensorFrame);

  // Remove outside limits in map frame
  if (!transformPointCloud(pointCloudSensorFrame, pointCloudMapFrame, generalParameters_.mapFrameId_)) {
    return false;
  }
  std::vector<PointCloudType::Ptr> pointClouds({pointCloudMapFrame, pointCloudSensorFrame});
  removePointsOutsideLimits(pointCloudMapFrame, pointClouds);

  // Compute variances
  return computeVariances(pointCloudSensorFrame, robotPoseCovariance, variances);
}

bool SensorProcessorBase::updateTransformations(const ros::Time& timeStamp) {
  try {

    geometry_msgs::TransformStamped transformGeom;
    transformGeom = transformBuffer_.lookupTransform(generalParameters_.mapFrameId_, sensorFrameId_, timeStamp, ros::Duration(1.0));
    transformationSensorToMap_ = tf2::transformToEigen(transformGeom);

    transformGeom = transformBuffer_.lookupTransform(generalParameters_.robotBaseFrameId_, sensorFrameId_, timeStamp,
                                                      ros::Duration(1.0));  // TODO(max): Why wrong direction?
    Eigen::Quaterniond rotationQuaternion;
    tf2::fromMsg(transformGeom.transform.rotation, rotationQuaternion);
    rotationBaseToSensor_.setMatrix(rotationQuaternion.toRotationMatrix());
    Eigen::Vector3d translationVector;
    tf2::fromMsg(transformGeom.transform.translation, translationVector);
    translationBaseToSensorInBaseFrame_.toImplementation() = translationVector;

    transformGeom = transformBuffer_.lookupTransform(generalParameters_.mapFrameId_, generalParameters_.robotBaseFrameId_,
                                                    timeStamp, ros::Duration(1.0));  // TODO(max): Why wrong direction?
    tf2::fromMsg(transformGeom.transform.rotation, rotationQuaternion);
    rotationMapToBase_.setMatrix(rotationQuaternion.toRotationMatrix());
    tf2::fromMsg(transformGeom.transform.translation, translationVector);
    translationMapToBaseInMapFrame_.toImplementation() = translationVector;

    if (!firstTfAvailable_) {
      firstTfAvailable_ = true;
    }

    return true;
  } catch (tf2::TransformException& ex) {
    if (!firstTfAvailable_) {
      return false;
    }
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool SensorProcessorBase::transformPointCloud(PointCloudType::ConstPtr pointCloud, PointCloudType::Ptr pointCloudTransformed,
                                              const std::string& targetFrame) {
  ros::Time timeStamp;
  timeStamp.fromNSec(1000 * pointCloud->header.stamp);
  const std::string inputFrameId(pointCloud->header.frame_id);

  try {
    geometry_msgs::TransformStamped transformGeom;
    transformGeom = transformBuffer_.lookupTransform(targetFrame, inputFrameId, timeStamp, ros::Duration(1.0));  // FIXME: missing 0.001 retry duration
    Eigen::Affine3d transform = tf2::transformToEigen(transformGeom);
    pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
    pointCloudTransformed->header.frame_id = targetFrame;

    ROS_DEBUG_THROTTLE(5, "Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
                      pointCloudTransformed->header.stamp / 1000.0);
  } catch (tf2::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  return true;
}

void SensorProcessorBase::removePointsOutsideLimits(PointCloudType::ConstPtr reference, std::vector<PointCloudType::Ptr>& pointClouds) {
  if (!std::isfinite(ignorePointsLowerThreshold_) && !std::isfinite(ignorePointsUpperThreshold_)) {
    return;
  }
  ROS_DEBUG("Limiting point cloud to the height interval of [%f, %f] relative to the robot base.", ignorePointsLowerThreshold_,
            ignorePointsUpperThreshold_);

  pcl::PassThrough<pcl::PointXYZRGBConfidenceRatio> passThroughFilter(true);
  passThroughFilter.setInputCloud(reference);
  passThroughFilter.setFilterFieldName("z");  // TODO(max): Should this be configurable?
  double relativeLowerThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsLowerThreshold_;
  double relativeUpperThreshold = translationMapToBaseInMapFrame_.z() + ignorePointsUpperThreshold_;
  passThroughFilter.setFilterLimits(relativeLowerThreshold, relativeUpperThreshold);
  pcl::IndicesPtr insideIndeces(new std::vector<int>);
  passThroughFilter.filter(*insideIndeces);

  for (auto& pointCloud : pointClouds) {
    pcl::ExtractIndices<pcl::PointXYZRGBConfidenceRatio> extractIndicesFilter;
    extractIndicesFilter.setInputCloud(pointCloud);
    extractIndicesFilter.setIndices(insideIndeces);
    PointCloudType tempPointCloud;
    extractIndicesFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }

  ROS_DEBUG("removePointsOutsideLimits() reduced point cloud to %i points.", (int)pointClouds[0]->size());
}

bool SensorProcessorBase::filterPointCloud(const PointCloudType::Ptr pointCloud) {
  PointCloudType tempPointCloud;

  // Remove nan points.
  std::vector<int> indices;
  if (!pointCloud->is_dense) {
    pcl::removeNaNFromPointCloud(*pointCloud, tempPointCloud, indices);
    tempPointCloud.is_dense = true;
    pointCloud->swap(tempPointCloud);
  }

  // Reduce points using VoxelGrid filter.
  if (applyVoxelGridFilter_) {
    pcl::VoxelGrid<pcl::PointXYZRGBConfidenceRatio> voxelGridFilter;
    voxelGridFilter.setInputCloud(pointCloud);
    double filter_size = sensorParameters_.at("voxelgrid_filter_size");
    voxelGridFilter.setLeafSize(filter_size, filter_size, filter_size);
    voxelGridFilter.filter(tempPointCloud);
    pointCloud->swap(tempPointCloud);
  }
  ROS_DEBUG_THROTTLE(2, "cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
  return true;
}

bool SensorProcessorBase::filterPointCloudSensorType(const PointCloudType::Ptr /*pointCloud*/) {
  return true;
}

} /* namespace elevation_mapping */
