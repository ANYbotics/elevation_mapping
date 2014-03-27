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
    : nodeHandle_(nodeHandle),
      sensorProcessor_(transformListener_)
{
  ROS_INFO("StarlETH elevation map node started.");
  readParameters();
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &ElevationMapping::pointCloudCallback, this);
  elevationMapRawPublisher_ = nodeHandle_.advertise<starleth_elevation_msg::ElevationMap>("elevation_map_raw", 1);
  elevationMapPublisher_ = nodeHandle_.advertise<starleth_elevation_msg::ElevationMap>("elevation_map", 1);
  robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_, 1);
  robotPoseCache_.connectInput(robotPoseSubscriber_);
  robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  mapUpdateTimer_ = nodeHandle_.createTimer(maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);
  fusionTriggerService_ = nodeHandle_.advertiseService("trigger_fusion", &ElevationMapping::fuseMap, this);
  submapService_ = nodeHandle_.advertiseService("get_submap", &ElevationMapping::getSubmap, this);
  initialize();
}

ElevationMapping::~ElevationMapping()
{

}

bool ElevationMapping::readParameters()
{
  // ElevationMapping parameters.
  nodeHandle_.param("point_cloud_topic", pointCloudTopic_, string("/depth_registered/points"));
  nodeHandle_.param("map_frame_id", parentFrameId_, string("/map"));
  nodeHandle_.param("elevation_map_frame_id", elevationMapFrameId_, string("/elevation_map"));
  nodeHandle_.param("track_point_frame_id", trackPointFrameId_, string("/odom"));
  nodeHandle_.param("track_point_x", trackPoint_.x(), 0.0);
  nodeHandle_.param("track_point_y", trackPoint_.y(), 0.0);
  nodeHandle_.param("track_point_z", trackPoint_.z(), 0.0);
  nodeHandle_.param("robot_pose_topic", robotPoseTopic_, string("/starleth/robot_state/pose"));

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
  nodeHandle_.param("min_variance", map_.minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", map_.maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_, 2.0);
  nodeHandle_.param("multi_height_noise", map_.multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("bigger_height_threshold_factor", map_.biggerHeightThresholdFactor_, 4.0);
  nodeHandle_.param("bigger_height_noise_factor", map_.biggerHeightNoiseFactor_, 2.0);
  nodeHandle_.param("min_horizontal_variance", map_.minHorizontalVariance_, 0.0);
  nodeHandle_.param("max_horizontal_variance", map_.maxHorizontalVariance_, 0.5);

  Eigen::Array2d length;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("resolution", resolution, 0.1);
  map_.setSize(length, resolution);

  // SensorProcessor parameters.
  nodeHandle_.param("base_frame_id", sensorProcessor_.baseFrameId_, string("/starleth/BASE"));
  nodeHandle_.param("sensor_cutoff_min_depth", sensorProcessor_.sensorCutoffMinDepth_, 0.2);
  ROS_ASSERT(sensorProcessor_.sensorCutoffMinDepth_ >= 0.0);
  nodeHandle_.param("sensor_cutoff_max_depth", sensorProcessor_.sensorCutoffMaxDepth_, 2.0);
  ROS_ASSERT(sensorProcessor_.sensorCutoffMaxDepth_ > sensorProcessor_.sensorCutoffMinDepth_);

  nodeHandle_.param("sensor_model_normal_factor_a", sensorProcessor_.sensorModelNormalFactorA_, 0.003);
  nodeHandle_.param("sensor_model_normal_factor_b", sensorProcessor_.sensorModelNormalFactorB_, 0.015);
  nodeHandle_.param("sensor_model_normal_factor_c", sensorProcessor_.sensorModelNormalFactorC_, 0.25);
  nodeHandle_.param("sensor_model_lateral_factor", sensorProcessor_.sensorModelLateralFactor_, 0.004);

  sensorProcessor_.mapFrameId_ = elevationMapFrameId_;
  sensorProcessor_.transformListenerTimeout_ = maxNoUpdateDuration_;
  sensorProcessor_.discretizationVariance_ = pow(resolution / 2.0, 2); // two-sigma

  return true;
}

bool ElevationMapping::initialize()
{
  ROS_INFO("StarlETH elevation map node initializing ... ");
  timeOfLastUpdate_ = Time::now();
  timeOfLastFusion_.fromSec(0.0);
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
  Matrix<float, Dynamic, sensorProcessor_.dimensionOfVariances> measurementVariances;
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

  // TODO remove.
//  std_srvs::Empty::Request request;
//  std_srvs::Empty::Response response;
//  fuseMap(request, response);

  // Publish raw elevation map.
  if (!publishRawElevationMap()) ROS_INFO("ElevationMap: Elevation map has not been broadcasted.");

  timeOfLastUpdate_ = time;
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

bool ElevationMapping::fuseMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  map_.fuse();
  timeOfLastFusion_ = timeOfLastUpdate_;
  publishElevationMap();
  return true;
}

bool ElevationMapping::broadcastElevationMapTransform(const ros::Time& time)
{
  tf::Transform tfTransform;
  poseEigenToTF(map_.getMapToParentTransform(), tfTransform);
  transformBroadcaster_.sendTransform(tf::StampedTransform(tfTransform, time, parentFrameId_, elevationMapFrameId_));
  ROS_DEBUG("Published transform for elevation map in parent frame at time %f.", time.toSec());
  return true;
}

bool ElevationMapping::updatePrediction(const ros::Time& time)
{
  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time < timeOfLastUpdate_)
  {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), timeOfLastUpdate_.toSec());
    return false;
  }

  // Variance from motion prediction
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage)
  {
    ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", time.toSec());
    return false;
  }

  Matrix<double, 6, 6> robotPoseCovariance = Map<const MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  Array2i size = map_.getBufferSize();
  MatrixXf varianceUpdate(size(0), size(1));
  MatrixXf horizontalVarianceUpdateX(size(0), size(1));
  MatrixXf horizontalVarianceUpdateY(size(0), size(1));
  mapUpdater_.computeUpdate(robotPoseCovariance, varianceUpdate, horizontalVarianceUpdateX, horizontalVarianceUpdateY);

  // Update map.
  map_.update(varianceUpdate, horizontalVarianceUpdateX, horizontalVarianceUpdateY);
  timeOfLastUpdate_ = time;

  return true;
}

// TODO Combine these two methods.
bool ElevationMapping::publishRawElevationMap()
{
  if (elevationMapRawPublisher_.getNumSubscribers() < 1) return false;

  starleth_elevation_msg::ElevationMap elevationMapMessage;

  elevationMapMessage.header.stamp = timeOfLastUpdate_;
  elevationMapMessage.header.frame_id = elevationMapFrameId_;
  elevationMapMessage.resolution = map_.getResolution();
  elevationMapMessage.lengthInX = map_.getLength()(0);
  elevationMapMessage.lengthInY = map_.getLength()(1);
  elevationMapMessage.outerStartIndex = map_.getBufferStartIndex()(0);
  elevationMapMessage.innerStartIndex = map_.getBufferStartIndex()(1);

  starleth_elevation_msg::matrixEigenToMultiArrayMessage(map_.getRawElevationData(), elevationMapMessage.elevation);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(map_.getRawVarianceData(), elevationMapMessage.variance);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(map_.getRawColorData(), elevationMapMessage.color);

  elevationMapRawPublisher_.publish(elevationMapMessage);

  ROS_DEBUG("Elevation map raw has been published.");

  return true;
}

bool ElevationMapping::publishElevationMap()
{
  if (elevationMapPublisher_.getNumSubscribers() < 1) return false;

  starleth_elevation_msg::ElevationMap elevationMapMessage;

  elevationMapMessage.header.stamp = timeOfLastFusion_;
  elevationMapMessage.header.frame_id = elevationMapFrameId_;
  elevationMapMessage.resolution = map_.getResolution();
  elevationMapMessage.lengthInX = map_.getLength()(0);
  elevationMapMessage.lengthInY = map_.getLength()(1);
  elevationMapMessage.outerStartIndex = map_.getBufferStartIndex()(0);
  elevationMapMessage.innerStartIndex = map_.getBufferStartIndex()(1);

  starleth_elevation_msg::matrixEigenToMultiArrayMessage(map_.getElevationData(), elevationMapMessage.elevation);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(map_.getVarianceData(), elevationMapMessage.variance);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(map_.getColorData(), elevationMapMessage.color);

  elevationMapPublisher_.publish(elevationMapMessage);

  ROS_DEBUG("Elevation map (fused) has been published.");

  return true;
}

bool ElevationMapping::updateMapLocation()
{
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = Time(0);
  trackPoint.point.x = trackPoint_.x();
  trackPoint.point.y = trackPoint_.y();
  trackPoint.point.z = trackPoint_.z();

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

void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - (Time::now() - timeOfLastUpdate_));
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace starleth_elevation_map */
