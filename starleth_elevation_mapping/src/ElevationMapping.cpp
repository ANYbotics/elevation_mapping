/*
 * ElevationMap.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "ElevationMapping.hpp"

// StarlETH Navigation
#include <ElevationMessageHelpers.hpp>
#include <starleth_elevation_msg/ElevationMap.h>
#include <EigenConversions.hpp>
#include <TransformationMath.hpp>

// ROS
#include <tf_conversions/tf_eigen.h>

// PCL
#include <pcl/ros/conversions.h>

using namespace std;
using namespace Eigen;
using namespace ros;
using namespace tf;
using namespace pcl;

namespace starleth_elevation_mapping {

ElevationMapping::ElevationMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("StarlETH elevation map node started.");
  parameters_.read(nodeHandle_);
  pointCloudSubscriber_ = nodeHandle_.subscribe(parameters_.pointCloudTopic_, 1, &ElevationMapping::pointCloudCallback, this);
  elevationMapPublisher_ = nodeHandle_.advertise<starleth_elevation_msg::ElevationMap>("elevation_map", 1);
  fusedElevationMapPublisher_ = nodeHandle_.advertise<starleth_elevation_msg::ElevationMap>("elevation_map_fused", 1);
  robotPoseSubscriber_.subscribe(nodeHandle_, parameters_.robotPoseTopic_, 1);
  robotPoseCache_.connectInput(robotPoseSubscriber_);
  robotPoseCache_.setCacheSize(parameters_.robotPoseCacheSize_);
  mapUpdateTimer_ = nodeHandle_.createTimer(parameters_.maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);
  submapService_ = nodeHandle_.advertiseService("get_subpart_of_elevation_map", &ElevationMapping::getSubmap, this);
  initialize();
}

ElevationMapping::~ElevationMapping()
{

}

bool ElevationMapping::ElevationMappingParameters::read(ros::NodeHandle& nodeHandle)
{
  nodeHandle.param("point_cloud_topic", pointCloudTopic_, string("/depth_registered/points_throttled"));
  nodeHandle.param("map_frame_id", parentFrameId_, string("/map"));
  nodeHandle.param("elevation_map_frame_id", elevationMapFrameId_, string("/elevation_map"));
  nodeHandle.param("track_point_frame_id", trackPointFrameId_, string("/odom"));
  nodeHandle.param("track_point_x", trackPoint_.x(), 0.5);
  nodeHandle.param("track_point_y", trackPoint_.y(), 0.0);
  nodeHandle.param("track_point_z", trackPoint_.z(), 0.0);
  nodeHandle.param("robot_pose_topic", robotPoseTopic_, string("/starleth/robot_state/pose"));
  nodeHandle.param("robot_pose_cache_size", robotPoseCacheSize_, 200);
//  nodeHandle.param("sensor_cutoff_min_depth", sensorCutoffMaxDepth_, 0.2);
//  nodeHandle.param("sensor_cutoff_max_depth", sensorCutoffMaxDepth_, 2.0);
//  nodeHandle.param("sensor_model_factor_a", sensorModelFactorA_, 0.003);
//  nodeHandle.param("sensor_model_factor_b", sensorModelFactorB_, 0.015);
//  nodeHandle.param("sensor_model_factor_c", sensorModelFactorC_, 0.25);
  // TODO
//  nodeHandle.param("length_in_x", length_(0), 1.5);
//  nodeHandle.param("length_in_y", length_(1), 1.5);
//  nodeHandle.param("resolution", resolution_, 0.1);
//  nodeHandle.param("min_variance", minVariance_, pow(0.003, 2));
//  nodeHandle.param("max_variance", maxVariance_, pow(0.03, 2));
//  nodeHandle.param("mahalanobis_distance_threshold", mahalanobisDistanceThreshold_, 2.0);
//  nodeHandle.param("multi_height_noise", multiHeightNoise_, pow(0.003, 2));
//  nodeHandle.param("bigger_height_threshold_factor", biggerHeightThresholdFactor_, 4.0);
//  nodeHandle.param("bigger_height_noise_factor", biggerHeightNoiseFactor_, 2.0);

  double minUpdateRate;
  nodeHandle.param("min_update_rate", minUpdateRate, 2.0);
  maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);

  double relocateRate;
  nodeHandle.param("relocate_rate", relocateRate, 3.0);
  if (relocateRate > 0.0)
    mapRelocateTimerDuration_.fromSec(1.0 / relocateRate);
  else
  {
    mapRelocateTimerDuration_.fromSec(0.0);
  }

  return checkValidity();
}

bool ElevationMapping::ElevationMappingParameters::checkValidity()
{
  ROS_ASSERT(sensorCutoffMinDepth_ >= 0.0);
  ROS_ASSERT(sensorCutoffMaxDepth_ > sensorCutoffMinDepth_);
  ROS_ASSERT(length_(0) > 0.0);
  ROS_ASSERT(length_(1) > 0.0);
  ROS_ASSERT(resolution_ > 0.0);
  ROS_ASSERT(minVariance_ >= 0.0);
  ROS_ASSERT(maxVariance_ > minVariance_);
  ROS_ASSERT(mahalanobisDistanceThreshold_ >= 0.0);
  ROS_ASSERT(multiHeightNoise_ >= 0.0);
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());
  ROS_ASSERT(robotPoseCacheSize_ >= 0);
  return true;
}

bool ElevationMapping::initialize()
{
  // TODO
  // map_.resize();

  timeOfLastUpdate_ = Time::now();
  broadcastElevationMapTransform(Time::now());
  Duration(1.0).sleep(); // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  ROS_INFO("StarlETH elevation map node initialized.");
  return true;
}

void ElevationMapping::pointCloudCallback(
    const sensor_msgs::PointCloud2& rawPointCloud)
{
  stopMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  PointCloud<PointXYZRGB>::Ptr pointCloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(rawPointCloud, *pointCloud);
  ROS_DEBUG("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  Time& time = pointCloud->header.stamp;
  updateMapLocation();
  if (!broadcastElevationMapTransform(time)) ROS_ERROR("ElevationMap: Broadcasting elevation map transform to parent failed.");
  if (!updatePrediction(time)) { ROS_ERROR("ElevationMap: Updating process noise failed."); return; }
  timeOfLastUpdate_ = time;
//  cleanPointCloud(pointCloud);
//  VectorXf measurementDistances;
//  getMeasurementDistances(pointCloud, measurementDistances);
//  if (!transformPointCloud(pointCloud, parameters_.elevationMapFrameId_)) ROS_ERROR("ElevationMap: Point cloud transform failed for time stamp %f.", time.toSec());
//  else if (!addToElevationMap(pointCloud, measurementDistances)) ROS_ERROR("ElevationMap: Adding point cloud to elevation map failed.");
//  cleanElevationMap();
//  generateFusedMap();
  if (!publishElevationMap()) ROS_INFO("ElevationMap: Elevation map has not been broadcasted.");
  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback(const ros::TimerEvent& timerEvent)
{
  ROS_WARN("Elevation map is updated without data from the sensor.");

  stopMapUpdateTimer();
  Time time = Time::now();
  if (!broadcastElevationMapTransform(time)) ROS_ERROR("ElevationMap: Broadcasting elevation map transform to parent failed.");
  if (!updatePrediction(time)) { ROS_ERROR("ElevationMap: Updating process noise failed."); return; }
  timeOfLastUpdate_ = time;
//  cleanElevationMap();
//  generateFusedMap();
  if (!publishElevationMap()) ROS_ERROR("ElevationMap: Elevation map has not been broadcasted.");
  resetMapUpdateTimer();
}

bool ElevationMapping::broadcastElevationMapTransform(const ros::Time& time)
{
  tf::Transform tfTransform;
//  poseEigenToTF(elevationMapToParentTransform_, tfTransform);
  transformBroadcaster_.sendTransform(tf::StampedTransform(tfTransform, time, parameters_.parentFrameId_, parameters_.elevationMapFrameId_));
  ROS_DEBUG("Published transform for elevation map in parent frame at time %f.", time.toSec());
  return true;
}

bool ElevationMapping::updatePrediction(const ros::Time& time)
{
  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

//  if (time < timeOfLastUpdate_)
//  {
//    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), timeOfLastUpdate_.toSec());
//    return false;
//  }
//
//  // Variance from motion prediction
//  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
//  if (!poseMessage)
//  {
//    ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", time.toSec());
//    return false;
//  }
//  Matrix<double, 6, 6> newRobotPoseCovariance = Map<const MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
//
//  Vector3d positionVariance = robotPoseCovariance_.diagonal().head(3);
//  Vector3d newPositionVariance = newRobotPoseCovariance.diagonal().head(3);
//
//  Vector3f variancePrediction = (newPositionVariance - positionVariance).cast<float>();
//
//  // Update map
//  varianceData_ = (varianceData_.array() + variancePrediction.z()).matrix();
//  horizontalVarianceDataX_ = (horizontalVarianceDataX_.array() + variancePrediction.x()).matrix();
//  horizontalVarianceDataY_ = (horizontalVarianceDataY_.array() + variancePrediction.y()).matrix();
//
//  // Make valid variances
//  varianceData_ = varianceData_.cwiseMax(0.0);
//  horizontalVarianceDataX_ = horizontalVarianceDataX_.cwiseMax(0.0);
//  horizontalVarianceDataY_ = horizontalVarianceDataY_.cwiseMax(0.0);
//
//  robotPoseCovariance_ = newRobotPoseCovariance;

  return true;
}

bool ElevationMapping::publishElevationMap()
{
  if (elevationMapPublisher_.getNumSubscribers() < 1) return false;

//  starleth_elevation_msg::ElevationMap elevationMapMessage;
//
//  elevationMapMessage.header.stamp = timeOfLastUpdate_;
//  elevationMapMessage.header.frame_id = parameters_.elevationMapFrameId_;
//  elevationMapMessage.resolution = parameters_.resolution_;
//  elevationMapMessage.lengthInX = parameters_.length_(0);
//  elevationMapMessage.lengthInY = parameters_.length_(1);
//  elevationMapMessage.outerStartIndex = circularBufferStartIndex_(0);
//  elevationMapMessage.innerStartIndex = circularBufferStartIndex_(1);
//
//  starleth_elevation_msg::matrixEigenToMultiArrayMessage(elevationData_, elevationMapMessage.elevation);
//  starleth_elevation_msg::matrixEigenToMultiArrayMessage(varianceData_, elevationMapMessage.variance);
//  starleth_elevation_msg::matrixEigenToMultiArrayMessage(colorData_, elevationMapMessage.color);
//
//  elevationMapPublisher_.publish(elevationMapMessage);

  ROS_DEBUG("Elevation map has been published.");

  return true;
}

bool ElevationMapping::updateMapLocation()
{
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = parameters_.trackPointFrameId_;
  trackPoint.header.stamp = Time(0);
  trackPoint.point.x = parameters_.trackPoint_.x();
  trackPoint.point.y = parameters_.trackPoint_.y();
  trackPoint.point.z = parameters_.trackPoint_.z();

  geometry_msgs::PointStamped trackPointTransformed;

  try
  {
    transformListener_.transformPoint(parameters_.parentFrameId_, trackPoint, trackPointTransformed);
  }
  catch (TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Vector3d position = Vector3d(trackPointTransformed.point.x,
                               trackPointTransformed.point.y,
                               trackPointTransformed.point.z);

  return map_.relocate(position);
}

bool ElevationMapping::getSubmap(starleth_elevation_msg::ElevationSubmap::Request& request, starleth_elevation_msg::ElevationSubmap::Response& response)
{
//  res.sum = req.a + req.b;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

bool ElevationMapping::publishFusedElevationMap(const Eigen::MatrixXf& fusedElevationData, const Eigen::MatrixXf& fusedVarianceData)
{
  if (fusedElevationMapPublisher_.getNumSubscribers() < 1) return false;

//  starleth_elevation_msg::ElevationMap fusedElevationMapMessage;
//
//  fusedElevationMapMessage.header.stamp = timeOfLastUpdate_;
//  fusedElevationMapMessage.header.frame_id = parameters_.elevationMapFrameId_;
//  fusedElevationMapMessage.resolution = parameters_.resolution_;
//  fusedElevationMapMessage.lengthInX = parameters_.length_(0);
//  fusedElevationMapMessage.lengthInY = parameters_.length_(1);
//  fusedElevationMapMessage.outerStartIndex = circularBufferStartIndex_(0);
//  fusedElevationMapMessage.innerStartIndex = circularBufferStartIndex_(1);
//
//  starleth_elevation_msg::matrixEigenToMultiArrayMessage(fusedElevationData, fusedElevationMapMessage.elevation);
//  starleth_elevation_msg::matrixEigenToMultiArrayMessage(fusedVarianceData, fusedElevationMapMessage.variance);
//  starleth_elevation_msg::matrixEigenToMultiArrayMessage(colorData_, fusedElevationMapMessage.color);
//
//  fusedElevationMapPublisher_.publish(fusedElevationMapMessage);

  ROS_DEBUG("Fused elevation map has been published.");

  return true;
}

void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  mapUpdateTimer_.setPeriod(parameters_.maxNoUpdateDuration_ - (Time::now() - timeOfLastUpdate_));
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace starleth_elevation_map */
