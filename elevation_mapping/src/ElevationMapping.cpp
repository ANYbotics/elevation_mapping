/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_mapping/ElevationMapping.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"
#include "elevation_map_msg/EigenConversions.hpp"
#include "elevation_map_msg/TransformationMath.hpp"

// PCL
#include <pcl/ros/conversions.h>

// Kindr
#include <kindr/poses/PoseEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>
#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/thirdparty/ros/RosEigen.hpp>

using namespace std;
using namespace Eigen;
using namespace ros;
using namespace tf;
using namespace pcl;
using namespace kindr::poses::eigen_impl;
using namespace kindr::phys_quant::eigen_impl;
using namespace kindr::rotations::eigen_impl;

namespace elevation_mapping {

ElevationMapping::ElevationMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      sensorProcessor_(transformListener_)
{
  ROS_INFO("Elevation mapping node started.");
  readParameters();
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &ElevationMapping::pointCloudCallback, this);
  elevationMapRawPublisher_ = nodeHandle_.advertise<elevation_map_msg::ElevationMap>("elevation_map_raw", 1);
  elevationMapPublisher_ = nodeHandle_.advertise<elevation_map_msg::ElevationMap>("elevation_map", 1);
  robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_, 1);
  robotPoseCache_.connectInput(robotPoseSubscriber_);
  robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  mapUpdateTimer_ = nodeHandle_.createTimer(maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);
  fusionTriggerService_ = nodeHandle_.advertiseService("trigger_fusion", &ElevationMapping::fuseEntireMap, this);
  submapService_ = nodeHandle_.advertiseService("get_submap", &ElevationMapping::getSubmap, this);
  initialize();
}

ElevationMapping::~ElevationMapping()
{

}

bool ElevationMapping::readParameters()
{
  // ElevationMapping parameters.
  nodeHandle_.param("point_cloud_topic", pointCloudTopic_, string("/points"));
  nodeHandle_.param("robot_pose_topic", robotPoseTopic_, string("/robot_state/pose"));
  nodeHandle_.param("parent_frame_id", parentFrameId_, string("/map"));
  nodeHandle_.param("track_point_frame_id", trackPointFrameId_, string("/robot"));
  nodeHandle_.param("track_point_x", trackPoint_.x(), 0.0);
  nodeHandle_.param("track_point_y", trackPoint_.y(), 0.0);
  nodeHandle_.param("track_point_z", trackPoint_.z(), 0.0);

  nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);
  ROS_ASSERT(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());

  double relocateRate;
  nodeHandle_.param("relocate_rate", relocateRate, 3.0);
  if (relocateRate > 0.0)
    mapRelocateTimerDuration_.fromSec(1.0 / relocateRate);
  else
  {
    mapRelocateTimerDuration_.fromSec(0.0);
  }

  // ElevationMap parameters.
  nodeHandle_.param("elevation_map_frame_id", map_.frameId_, string("/elevation_map"));

  Eigen::Array2d length;
  kindr::phys_quant::eigen_impl::Position3D position;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position(0), 0.0);
  nodeHandle_.param("position_y", position(1), 0.0);
  nodeHandle_.param("resolution", resolution, 0.01);
  map_.setGeometry(length, position, resolution);

  nodeHandle_.param("min_variance", map_.minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", map_.maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", map_.multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", map_.minHorizontalVariance_, pow(resolution / 2.0, 2)); // two-sigma
  nodeHandle_.param("max_horizontal_variance", map_.maxHorizontalVariance_, 0.5);

  // SensorProcessor parameters.
  nodeHandle_.param("base_frame_id", sensorProcessor_.baseFrameId_, string("/robot"));
  nodeHandle_.param("sensor_cutoff_min_depth", sensorProcessor_.sensorCutoffMinDepth_, 0.2);
  ROS_ASSERT(sensorProcessor_.sensorCutoffMinDepth_ >= 0.0);
  nodeHandle_.param("sensor_cutoff_max_depth", sensorProcessor_.sensorCutoffMaxDepth_, 2.0);
  ROS_ASSERT(sensorProcessor_.sensorCutoffMaxDepth_ > sensorProcessor_.sensorCutoffMinDepth_);

  nodeHandle_.param("sensor_model_normal_factor_a", sensorProcessor_.sensorModelNormalFactorA_, 0.003);
  nodeHandle_.param("sensor_model_normal_factor_b", sensorProcessor_.sensorModelNormalFactorB_, 0.015);
  nodeHandle_.param("sensor_model_normal_factor_c", sensorProcessor_.sensorModelNormalFactorC_, 0.25);
  nodeHandle_.param("sensor_model_lateral_factor", sensorProcessor_.sensorModelLateralFactor_, 0.004);

  sensorProcessor_.mapFrameId_ = map_.frameId_;
  sensorProcessor_.transformListenerTimeout_ = maxNoUpdateDuration_;

  return true;
}

bool ElevationMapping::initialize()
{
  ROS_INFO("Elevation mapping node initializing ... ");
  broadcastElevationMapTransform(Time::now());
  Duration(1.0).sleep(); // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  ROS_INFO("Done.");
  return true;
}

void ElevationMapping::pointCloudCallback(
    const sensor_msgs::PointCloud2& rawPointCloud)
{
  stopMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  PointCloud<PointXYZRGB>::Ptr pointCloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(rawPointCloud, *pointCloud);
  Time& time = pointCloud->header.stamp;
  ROS_DEBUG("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Update map location.
  updateMapLocation();
  if (!broadcastElevationMapTransform(time)) ROS_ERROR("ElevationMap: Broadcasting elevation map transform to parent failed.");

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    ROS_ERROR("ElevationMap: Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Get robot pose covariance matrix at timestamp of point cloud.
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage)
  {
    ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", time.toSec());
    return;
  }

  Matrix<double, 6, 6> robotPoseCovariance = Map<const MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);

  // Process point cloud.
  PointCloud<PointXYZRGB>::Ptr pointCloudProcessed(new PointCloud<PointXYZRGB>);
  VectorXf measurementVariances;
  if(!sensorProcessor_.process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances))
  {
    ROS_ERROR("ElevationMap: Point cloud could not be processed.");
    resetMapUpdateTimer();
    return;
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances))
  {
    ROS_ERROR("ElevationMap: Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish raw elevation map.
  if (!publishRawElevationMap()) ROS_INFO("ElevationMap: Elevation map has not been broadcasted.");

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback(const ros::TimerEvent& timerEvent)
{
  ROS_WARN("Elevation map is updated without data from the sensor.");

  stopMapUpdateTimer();
  Time time = Time::now();

  if (!broadcastElevationMapTransform(time)) ROS_ERROR("ElevationMap: Broadcasting elevation map transform to parent failed.");

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    ROS_ERROR("ElevationMap: Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish raw elevation map.
  if (!publishRawElevationMap()) ROS_ERROR("ElevationMap: Elevation map has not been broadcasted.");

  resetMapUpdateTimer();
}

bool ElevationMapping::fuseEntireMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  map_.fuseAll();
  publishElevationMap();
  return true;
}

bool ElevationMapping::broadcastElevationMapTransform(const ros::Time& time)
{
  tf::Transform tfTransform;
  convertToRosTf(map_.getPose(), tfTransform);
  transformBroadcaster_.sendTransform(tf::StampedTransform(tfTransform, time, parentFrameId_, map_.frameId_));
  ROS_DEBUG("Published transform for elevation map in parent frame at time %f.", time.toSec());
  return true;
}

bool ElevationMapping::updatePrediction(const ros::Time& time)
{
  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time < map_.getTimeOfLastUpdate())
  {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  }

  // Get robot pose at requested time.
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage)
  {
    ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", time.toSec());
    return false;
  }

  kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD robotPose;
  kindr::poses::eigen_impl::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  Matrix<double, 6, 6> robotPoseCovariance = Map<const MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::publishRawElevationMap()
{
  if (elevationMapRawPublisher_.getNumSubscribers() < 1) return false;

  elevation_map_msg::ElevationMap elevationMapMessage;
  elevationMapMessage.header.stamp = map_.getTimeOfLastUpdate();
  addHeaderDataToElevationMessage(elevationMapMessage);

  elevation_map_msg::matrixEigenToMultiArrayMessage(map_.getRawElevationData(), elevationMapMessage.elevation);
  elevation_map_msg::matrixEigenToMultiArrayMessage(map_.getRawVarianceData(), elevationMapMessage.variance);
  elevation_map_msg::matrixEigenToMultiArrayMessage(map_.getRawColorData(), elevationMapMessage.color);

  elevationMapRawPublisher_.publish(elevationMapMessage);

  ROS_DEBUG("Elevation map raw has been published.");

  return true;
}

bool ElevationMapping::publishElevationMap()
{
  if (elevationMapPublisher_.getNumSubscribers() < 1) return false;

  elevation_map_msg::ElevationMap elevationMapMessage;
  elevationMapMessage.header.stamp = map_.getTimeOfLastFusion();
  addHeaderDataToElevationMessage(elevationMapMessage);

  elevation_map_msg::matrixEigenToMultiArrayMessage(map_.getElevationData(), elevationMapMessage.elevation);
  elevation_map_msg::matrixEigenToMultiArrayMessage(map_.getVarianceData(), elevationMapMessage.variance);
  elevation_map_msg::matrixEigenToMultiArrayMessage(map_.getColorData(), elevationMapMessage.color);

  elevationMapPublisher_.publish(elevationMapMessage);

  ROS_DEBUG("Elevation map (fused) has been published.");

  return true;
}

void ElevationMapping::addHeaderDataToElevationMessage(elevation_map_msg::ElevationMap& elevationMapMessage)
{
  elevationMapMessage.header.frame_id = map_.frameId_;
  elevationMapMessage.lengthInX = map_.getLength()(0);
  elevationMapMessage.lengthInY = map_.getLength()(1);
  elevationMapMessage.position.x = map_.getPosition().x();
  elevationMapMessage.position.y = map_.getPosition().y();
  elevationMapMessage.resolution = map_.getResolution();
  elevationMapMessage.outerStartIndex = map_.getBufferStartIndex()(0);
  elevationMapMessage.innerStartIndex = map_.getBufferStartIndex()(1);
}

bool ElevationMapping::updateMapLocation()
{
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = Time(0);
  convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::PointStamped trackPointTransformed;

  try
  {
    transformListener_.transformPoint(parentFrameId_, trackPoint, trackPointTransformed);
  }
  catch (TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Position3D position;
  convertFromRosGeometryMsg(trackPointTransformed.point, position);

  return map_.relocate(position);
}

bool ElevationMapping::getSubmap(elevation_map_msg::GetSubmap::Request& request, elevation_map_msg::GetSubmap::Response& response)
{
  // Request
  Vector2d requestedSubmapPosition(request.positionX, request.positionY);
  Array2d requestedSubmapLength(request.lengthInX, request.lengthInY);

  ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));

  // Response
  MatrixXf submapElevation, submapVariance, submapColor;
  Vector2d submapPosition;
  Array2d submapLength;
  Array2i submapBufferSize;
  Array2i requestedIndexInSubmap;

  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);
  map_.getSubmap(submapElevation, submapPosition, submapLength, submapBufferSize, requestedIndexInSubmap, map_.getElevationData(), requestedSubmapPosition, requestedSubmapLength);
  map_.getSubmap(submapVariance, submapPosition, submapLength, submapBufferSize, requestedIndexInSubmap, map_.getVarianceData(), requestedSubmapPosition, requestedSubmapLength);
  // TODO Add color for submaps.
//  map_.getSubmap(submapColor, submapPosition, submapLength, submapBufferSize, requestedIndexInSubmap, map_.getColorData(), requestedSubmapPosition, requestedSubmapLength);

  response.elevation_map.header.stamp = map_.getTimeOfLastFusion();
  response.elevation_map.header.frame_id = map_.frameId_;
  response.elevation_map.lengthInX = submapLength(0);
  response.elevation_map.lengthInY = submapLength(1);
  response.elevation_map.position.x = submapPosition.x();
  response.elevation_map.position.y = submapPosition.y();
  response.elevation_map.resolution = map_.getResolution();
  response.elevation_map.outerStartIndex = 0;
  response.elevation_map.innerStartIndex = 0;

  elevation_map_msg::matrixEigenToMultiArrayMessage(submapElevation, response.elevation_map.elevation);
  elevation_map_msg::matrixEigenToMultiArrayMessage(submapVariance, response.elevation_map.variance);

  ROS_DEBUG("Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());

  return true;
}

void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - (Time::now() - map_.getTimeOfLastUpdate()));
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace */
