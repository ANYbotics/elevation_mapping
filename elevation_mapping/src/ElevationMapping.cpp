/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <cmath>
#include <memory>
#include <string>

#include <grid_map_msgs/GridMap.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/ElevationMapping.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"

namespace elevation_mapping {

ElevationMapping::ElevationMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      inputSources_(nodeHandle_),
      map_(nodeHandle),
      robotMotionMapUpdater_(nodeHandle),
      receivedFirstMatchingPointcloudAndPose_(false) {
#ifndef NDEBUG
  // Print a warning if built in debug.
  ROS_WARN("CMake Build Type is 'Debug'. Change to 'Release' for better performance.");
#endif

  ROS_INFO("Elevation mapping node started.");

  readParameters();
  setupSubscribers();
  setupServices();
  setupTimers();

  initialize();

  ROS_INFO("Successfully launched node.");
}

void ElevationMapping::setupSubscribers() {  // Handle deprecated point_cloud_topic and input_sources configuration.
  const bool configuredInputSources = inputSources_.configureFromRos("input_sources");
  const bool hasDeprecatedPointcloudTopic = nodeHandle_.hasParam("point_cloud_topic");
  if (hasDeprecatedPointcloudTopic) {
    ROS_WARN("Parameter 'point_cloud_topic' is deprecated, please use 'input_sources' instead.");
  }
  if (!configuredInputSources && hasDeprecatedPointcloudTopic) {
    pointCloudSubscriber_ = nodeHandle_.subscribe<sensor_msgs::PointCloud2>(
        parameters_.pointCloudTopic_, 1, [&](const auto& msg) { pointCloudCallback(msg, true, sensorProcessor_); });
  }
  if (configuredInputSources) {
    inputSources_.registerCallbacks(*this, make_pair("pointcloud", &ElevationMapping::pointCloudCallback));
  }

  if (!parameters_.robotPoseTopic_.empty()) {
    robotPoseSubscriber_.subscribe(nodeHandle_, parameters_.robotPoseTopic_, 1);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(parameters_.robotPoseCacheSize_);
  } else {
    parameters_.ignoreRobotMotionUpdates_ = true;
  }
}

void ElevationMapping::setupServices() {
  // Multi-threading for fusion.
  ros::AdvertiseServiceOptions advertiseServiceOptionsForTriggerFusion = ros::AdvertiseServiceOptions::create<std_srvs::Empty>(
      "trigger_fusion", [&](auto& req, auto& res) { return fuseEntireMapServiceCallback(req, res); }, ros::VoidConstPtr(),
      &fusionServiceQueue_);
  fusionTriggerService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForTriggerFusion);

  ros::AdvertiseServiceOptions advertiseServiceOptionsForGetFusedSubmap = ros::AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_submap", [&](auto& req, auto& res) { return getFusedSubmapServiceCallback(req, res); }, ros::VoidConstPtr(),
      &fusionServiceQueue_);
  fusedSubmapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetFusedSubmap);

  ros::AdvertiseServiceOptions advertiseServiceOptionsForGetRawSubmap = ros::AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_raw_submap", [&](auto& req, auto& res) { return getRawSubmapServiceCallback(req, res); }, ros::VoidConstPtr(),
      &fusionServiceQueue_);
  rawSubmapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetRawSubmap);

  clearMapService_ = nodeHandle_.advertiseService("clear_map", &ElevationMapping::clearMapServiceCallback, this);
  enableUpdatesService_ = nodeHandle_.advertiseService("enable_updates", &ElevationMapping::enableUpdatesServiceCallback, this);
  disableUpdatesService_ = nodeHandle_.advertiseService("disable_updates", &ElevationMapping::disableUpdatesServiceCallback, this);
  maskedReplaceService_ = nodeHandle_.advertiseService("masked_replace", &ElevationMapping::maskedReplaceServiceCallback, this);
  saveMapService_ = nodeHandle_.advertiseService("save_map", &ElevationMapping::saveMapServiceCallback, this);
  loadMapService_ = nodeHandle_.advertiseService("load_map", &ElevationMapping::loadMapServiceCallback, this);
}

void ElevationMapping::setupTimers() {
  mapUpdateTimer_ = nodeHandle_.createTimer(parameters_.maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);

  if (!parameters_.fusedMapPublishTimerDuration_.isZero()) {
    ros::TimerOptions timerOptions = ros::TimerOptions(
        parameters_.fusedMapPublishTimerDuration_, [this](auto&& PH1) { publishFusedMapCallback(PH1); }, &fusionServiceQueue_, false,
        false);
    fusedMapPublishTimer_ = nodeHandle_.createTimer(timerOptions);
  }

  // Multi-threading for visibility cleanup. Visibility clean-up does not help when continuous clean-up is enabled.
  if (map_.parameters_.enableVisibilityCleanup_ && !parameters_.visibilityCleanupTimerDuration_.isZero() &&
      !map_.parameters_.enableContinuousCleanup_) {
    ros::TimerOptions timerOptions = ros::TimerOptions(
        parameters_.visibilityCleanupTimerDuration_, [this](auto&& PH1) { visibilityCleanupCallback(PH1); }, &visibilityCleanupQueue_,
        false, false);
    visibilityCleanupTimer_ = nodeHandle_.createTimer(timerOptions);
  }
}

ElevationMapping::~ElevationMapping() {
  // Shutdown all services.

  {  // Fusion Service Queue
    rawSubmapService_.shutdown();
    fusionTriggerService_.shutdown();
    fusedSubmapService_.shutdown();
    fusedMapPublishTimer_.stop();

    fusionServiceQueue_.disable();
    fusionServiceQueue_.clear();
  }

  {  // Visibility cleanup queue
    visibilityCleanupTimer_.stop();

    visibilityCleanupQueue_.disable();
    visibilityCleanupQueue_.clear();
  }

  nodeHandle_.shutdown();

  // Join threads.
  if (fusionServiceThread_.joinable()) {
    fusionServiceThread_.join();
  }
  if (visibilityCleanupThread_.joinable()) {
    visibilityCleanupThread_.join();
  }
}

bool ElevationMapping::readParameters() {
  // ElevationMapping parameters.
  nodeHandle_.param("point_cloud_topic", parameters_.pointCloudTopic_, std::string("/points"));
  nodeHandle_.param("robot_pose_with_covariance_topic", parameters_.robotPoseTopic_, std::string("/pose"));
  nodeHandle_.param("track_point_frame_id", parameters_.trackPointFrameId_, std::string("/robot"));
  nodeHandle_.param("track_point_x", parameters_.trackPoint_.x(), 0.0);
  nodeHandle_.param("track_point_y", parameters_.trackPoint_.y(), 0.0);
  nodeHandle_.param("track_point_z", parameters_.trackPoint_.z(), 0.0);

  nodeHandle_.param("robot_pose_cache_size", parameters_.robotPoseCacheSize_, 200);
  ROS_ASSERT(parameters_.robotPoseCacheSize_ >= 0);

  double minUpdateRate{2.0};
  nodeHandle_.param("min_update_rate", minUpdateRate, minUpdateRate);
  if (minUpdateRate == 0.0) {
    parameters_.maxNoUpdateDuration_.fromSec(0.0);
    ROS_WARN("Rate for publishing the map is zero.");
  } else {
    parameters_.maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  }
  ROS_ASSERT(!parameters_.maxNoUpdateDuration_.isZero());

  double timeTolerance{0.0};
  nodeHandle_.param("time_tolerance", timeTolerance, timeTolerance);
  parameters_.timeTolerance_.fromSec(timeTolerance);

  double fusedMapPublishingRate{1.0};
  nodeHandle_.param("fused_map_publishing_rate", fusedMapPublishingRate, fusedMapPublishingRate);
  if (fusedMapPublishingRate == 0.0) {
    parameters_.fusedMapPublishTimerDuration_.fromSec(0.0);
    ROS_WARN(
        "Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is "
        "called.");
  } else if (std::isinf(fusedMapPublishingRate)) {
    parameters_.isContinuouslyFusing_ = true;
    parameters_.fusedMapPublishTimerDuration_.fromSec(0.0);
  } else {
    parameters_.fusedMapPublishTimerDuration_.fromSec(1.0 / fusedMapPublishingRate);
  }

  double visibilityCleanupRate{1.0};
  nodeHandle_.param("visibility_cleanup_rate", visibilityCleanupRate, visibilityCleanupRate);
  if (visibilityCleanupRate == 0.0) {
    parameters_.visibilityCleanupTimerDuration_.fromSec(0.0);
    ROS_WARN("Rate for visibility cleanup is zero and therefore disabled.");
  } else {
    parameters_.visibilityCleanupTimerDuration_.fromSec(1.0 / visibilityCleanupRate);
    map_.parameters_.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }

  // ElevationMap parameters. TODO Move this to the elevation map class.
  nodeHandle_.param("map_frame_id", parameters_.mapFrameId_, std::string("/map"));
  map_.setFrameId(parameters_.mapFrameId_);

  grid_map::Length length;
  grid_map::Position position;
  double resolution{0.01};
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position.x(), 0.0);
  nodeHandle_.param("position_y", position.y(), 0.0);
  nodeHandle_.param("resolution", resolution, resolution);
  map_.setGeometry(length, resolution, position);

  nodeHandle_.param("min_variance", map_.parameters_.minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", map_.parameters_.maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", map_.parameters_.mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", map_.parameters_.multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", map_.parameters_.minHorizontalVariance_, pow(resolution / 2.0, 2));  // two-sigma
  nodeHandle_.param("max_horizontal_variance", map_.parameters_.maxHorizontalVariance_, 0.5);
  nodeHandle_.param("underlying_map_topic", map_.parameters_.underlyingMapTopic_, std::string());
  nodeHandle_.param("enable_visibility_cleanup", map_.parameters_.enableVisibilityCleanup_, true);
  nodeHandle_.param("enable_continuous_cleanup", map_.parameters_.enableContinuousCleanup_, false);
  nodeHandle_.param("scanning_duration", map_.parameters_.scanningDuration_, 1.0);
  nodeHandle_.param("masked_replace_service_mask_layer_name", parameters_.maskedReplaceServiceMaskLayerName_, std::string("mask"));

  // Settings for initializing elevation map
  nodeHandle_.param("initialize_elevation_map", parameters_.initializeElevationMap_, false);
  nodeHandle_.param("initialization_method", parameters_.initializationMethod_, 0);
  nodeHandle_.param("length_in_x_init_submap", parameters_.lengthInXInitSubmap_, 1.2);
  nodeHandle_.param("length_in_y_init_submap", parameters_.lengthInYInitSubmap_, 1.8);
  nodeHandle_.param("margin_init_submap", parameters_.marginInitSubmap_, 0.3);
  nodeHandle_.param("init_submap_height_offset", parameters_.initSubmapHeightOffset_, 0.0);
  nodeHandle_.param("target_frame_init_submap", parameters_.targetFrameInitSubmap_, std::string("/footprint"));

  // SensorProcessor parameters. Deprecated, use the sensorProcessor from within input sources instead!
  std::string sensorType;
  nodeHandle_.param("sensor_processor/type", sensorType, std::string("structured_light"));

  SensorProcessorBase::GeneralParameters generalSensorProcessorConfig{nodeHandle_.param("robot_base_frame_id", std::string("/robot")),
                                                                      parameters_.mapFrameId_};
  if (sensorType == "structured_light") {
    sensorProcessor_ = std::make_unique<StructuredLightSensorProcessor>(nodeHandle_, generalSensorProcessorConfig);
  } else if (sensorType == "stereo") {
    sensorProcessor_ = std::make_unique<StereoSensorProcessor>(nodeHandle_, generalSensorProcessorConfig);
  } else if (sensorType == "laser") {
    sensorProcessor_ = std::make_unique<LaserSensorProcessor>(nodeHandle_, generalSensorProcessorConfig);
  } else if (sensorType == "perfect") {
    sensorProcessor_ = std::make_unique<PerfectSensorProcessor>(nodeHandle_, generalSensorProcessorConfig);
  } else {
    ROS_ERROR("The sensor type %s is not available.", sensorType.c_str());
  }
  if (!sensorProcessor_->readParameters()) {
    return false;
  }
  if (!robotMotionMapUpdater_.readParameters()) {
    return false;
  }

  return true;
}

bool ElevationMapping::initialize() {
  ROS_INFO("Elevation mapping node initializing ... ");
  fusionServiceThread_ = boost::thread(&ElevationMapping::runFusionServiceThread, this);
  ros::Duration(1.0).sleep();  // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  fusedMapPublishTimer_.start();
  visibilityCleanupThread_ = boost::thread([this] { visibilityCleanupThread(); });
  visibilityCleanupTimer_.start();
  initializeElevationMap();
  return true;
}

void ElevationMapping::runFusionServiceThread() {
  ros::Rate loopRate(20);

  while (nodeHandle_.ok()) {
    fusionServiceQueue_.callAvailable();

    // Sleep until the next execution.
    loopRate.sleep();
  }
}

void ElevationMapping::visibilityCleanupThread() {
  ros::Rate loopRate(20);

  while (nodeHandle_.ok()) {
    visibilityCleanupQueue_.callAvailable();

    // Sleep until the next execution.
    loopRate.sleep();
  }
}

void ElevationMapping::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg, bool publishPointCloud,
                                          const SensorProcessorBase::Ptr& sensorProcessor_) {
  ROS_DEBUG("Processing data from: %s", pointCloudMsg->header.frame_id.c_str());
  if (!parameters_.updatesEnabled_) {
    ROS_WARN_THROTTLE(10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    if (publishPointCloud) {
      map_.setTimestamp(ros::Time::now());
      map_.postprocessAndPublishRawElevationMap();
    }
    return;
  }

  // Check if point cloud has corresponding robot pose at the beginning
  if (!receivedFirstMatchingPointcloudAndPose_) {
    const double oldestPoseTime = robotPoseCache_.getOldestTime().toSec();
    const double currentPointCloudTime = pointCloudMsg->header.stamp.toSec();

    if (currentPointCloudTime < oldestPoseTime) {
      ROS_WARN_THROTTLE(5, "No corresponding point cloud and pose are found. Waiting for first match. (Warning message is throttled, 5s.)");
      return;
    } else {
      ROS_INFO("First corresponding point cloud and pose found, elevation mapping started. ");
      receivedFirstMatchingPointcloudAndPose_ = true;
    }
  }

  stopMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO(max): Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(*pointCloudMsg, pcl_pc);

  PointCloudType::Ptr pointCloud(new PointCloudType);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_.fromNSec(1000 * pointCloud->header.stamp);

  ROS_DEBUG("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!parameters_.ignoreRobotMotionUpdates_) {
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage =
        robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
      if (robotPoseCache_.getOldestTime().toSec() > lastPointCloudUpdateTime_.toSec()) {
        ROS_ERROR("The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().toSec(),
                  lastPointCloudUpdateTime_.toSec());
      } else {
        ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
      }
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  PointCloudType::Ptr pointCloudProcessed(new PointCloudType);
  Eigen::VectorXf measurementVariances;
  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances,
                                 pointCloudMsg->header.frame_id)) {
    if (!sensorProcessor_->isTfAvailableInBuffer()) {
      ROS_INFO_THROTTLE(10, "Waiting for tf transformation to be available. (Message is throttled, 10s.)");
      return;
    }
    ROS_ERROR_THROTTLE(10, "Point cloud could not be processed. (Throttled 10s)");
    resetMapUpdateTimer();
    return;
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Clear the map if continuous clean-up was enabled.
  if (map_.parameters_.enableContinuousCleanup_) {
    ROS_DEBUG("Clearing elevation map before adding new point cloud.");
    map_.clear();
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_,
                Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    ROS_ERROR("Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  if (publishPointCloud) {
    // Publish elevation map.
    map_.postprocessAndPublishRawElevationMap();
    if (isFusingEnabled()) {
      map_.fuseAll();
      map_.publishFusedElevationMap();
    }
  }

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback(const ros::TimerEvent& /*unused*/) {
  if (!parameters_.updatesEnabled_) {
    ROS_WARN_THROTTLE(10, "Updating of elevation map is disabled. (Warning message is throttled, 10s.)");
    map_.setTimestamp(ros::Time::now());
    map_.postprocessAndPublishRawElevationMap();
    return;
  }

  ros::Time time = ros::Time::now();
  if ((lastPointCloudUpdateTime_ - time) <=
      parameters_.maxNoUpdateDuration_) {  // there were updates from sensordata, no need to force an update.
    return;
  }
  ROS_WARN_THROTTLE(5, "Elevation map is updated without data from the sensor. (Warning message is throttled, 5s.)");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  stopMapUpdateTimer();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_.postprocessAndPublishRawElevationMap();
  if (isFusingEnabled()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();
}

void ElevationMapping::publishFusedMapCallback(const ros::TimerEvent& /*unused*/) {
  if (!map_.hasFusedMapSubscribers()) {
    return;
  }
  ROS_DEBUG("Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback(const ros::TimerEvent& /*unused*/) {
  ROS_DEBUG("Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_.visibilityCleanup(ros::Time(lastPointCloudUpdateTime_));
}

bool ElevationMapping::fuseEntireMapServiceCallback(std_srvs::Empty::Request& /*unused*/, std_srvs::Empty::Response& /*unused*/) {
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
  return true;
}

bool ElevationMapping::isFusingEnabled() {
  return parameters_.isContinuouslyFusing_ && map_.hasFusedMapSubscribers();
}

bool ElevationMapping::updatePrediction(const ros::Time& time) {
  if (parameters_.ignoreRobotMotionUpdates_) {
    return true;
  }

  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time + parameters_.timeTolerance_ < map_.getTimeOfLastUpdate()) {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    ROS_DEBUG("Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(),
              map_.getTimeOfLastUpdate().toSec());
    return true;
  }

  // Get robot pose at requested time.
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    // Tell the user that either for the timestamp no pose is available or that the buffer is possibly empty
    if (robotPoseCache_.getOldestTime().toSec() > lastPointCloudUpdateTime_.toSec()) {
      ROS_ERROR("The oldest pose available is at %f, requested pose at %f", robotPoseCache_.getOldestTime().toSec(),
                lastPointCloudUpdateTime_.toSec());
    } else {
      ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
    }
    return false;
  }

  kindr::HomTransformQuatD robotPose;
  kindr_ros::convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance =
      Eigen::Map<const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation() {
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = parameters_.trackPointFrameId_;
  trackPoint.header.stamp = ros::Time(0);
  kindr_ros::convertToRosGeometryMsg(parameters_.trackPoint_, trackPoint.point);
  geometry_msgs::PointStamped trackPointTransformed;

  try {
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
  } catch (tf::TransformException& ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  kindr::Position3D position3d;
  kindr_ros::convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_.move(position);
  return true;
}

bool ElevationMapping::getFusedSubmapServiceCallback(grid_map_msgs::GetGridMap::Request& request,
                                                     grid_map_msgs::GetGridMap::Response& response) {
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(),
            requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess{false};
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }

  ROS_DEBUG("Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());
  return isSuccess;
}

bool ElevationMapping::getRawSubmapServiceCallback(grid_map_msgs::GetGridMap::Request& request,
                                                   grid_map_msgs::GetGridMap::Response& response) {
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  grid_map::Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(),
            requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  bool isSuccess{false};
  grid_map::Index index;
  grid_map::GridMap subMap = map_.getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    grid_map::GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    std::vector<std::string> layers;
    for (const std::string& layer : request.layers) {
      layers.push_back(layer);
    }
    grid_map::GridMapRosConverter::toMessage(subMap, layers, response.map);
  }
  return isSuccess;
}

bool ElevationMapping::disableUpdatesServiceCallback(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/) {
  ROS_INFO("Disabling updates.");
  parameters_.updatesEnabled_ = false;
  return true;
}

bool ElevationMapping::enableUpdatesServiceCallback(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/) {
  ROS_INFO("Enabling updates.");
  parameters_.updatesEnabled_ = true;
  return true;
}

bool ElevationMapping::initializeElevationMap() {
  if (parameters_.initializeElevationMap_) {
    if (static_cast<elevation_mapping::InitializationMethods>(parameters_.initializationMethod_) ==
        elevation_mapping::InitializationMethods::PlanarFloorInitializer) {
      tf::StampedTransform transform;

      // Listen to transform between mapFrameId_ and targetFrameInitSubmap_ and use z value for initialization
      try {
        transformListener_.waitForTransform(parameters_.mapFrameId_, parameters_.targetFrameInitSubmap_, ros::Time(0), ros::Duration(5.0));
        transformListener_.lookupTransform(parameters_.mapFrameId_, parameters_.targetFrameInitSubmap_, ros::Time(0), transform);
        ROS_DEBUG_STREAM("Initializing with x: " << transform.getOrigin().x() << " y: " << transform.getOrigin().y()
                                                 << " z: " << transform.getOrigin().z());

        const grid_map::Position positionRobot(transform.getOrigin().x(), transform.getOrigin().y());

        // Move map before we apply the height values. This prevents unwanted behavior from intermediate move() calls in
        // updateMapLocation().
        map_.move(positionRobot);

        map_.setRawSubmapHeight(positionRobot, transform.getOrigin().z() + parameters_.initSubmapHeightOffset_,
                                parameters_.lengthInXInitSubmap_, parameters_.lengthInYInitSubmap_, parameters_.marginInitSubmap_);
        return true;
      } catch (tf::TransformException& ex) {
        ROS_DEBUG("%s", ex.what());
        ROS_WARN("Could not initialize elevation map with constant height. (This warning can be ignored if TF tree is not available.)");
        return false;
      }
    }
  }
  return true;
}

bool ElevationMapping::clearMapServiceCallback(std_srvs::Empty::Request& /*request*/, std_srvs::Empty::Response& /*response*/) {
  ROS_INFO("Clearing map...");
  bool success = map_.clear();
  success &= initializeElevationMap();
  ROS_INFO("Map cleared.");

  return success;
}

bool ElevationMapping::maskedReplaceServiceCallback(grid_map_msgs::SetGridMap::Request& request,
                                                    grid_map_msgs::SetGridMap::Response& /*response*/) {
  ROS_INFO("Masked replacing of map.");
  grid_map::GridMap sourceMap;
  grid_map::GridMapRosConverter::fromMessage(request.map, sourceMap);

  // Use the supplied mask or do not use a mask
  grid_map::Matrix mask;
  if (sourceMap.exists(parameters_.maskedReplaceServiceMaskLayerName_)) {
    mask = sourceMap[parameters_.maskedReplaceServiceMaskLayerName_];
  } else {
    mask = Eigen::MatrixXf::Ones(sourceMap.getSize()(0), sourceMap.getSize()(1));
  }

  boost::recursive_mutex::scoped_lock scopedLockRawData(map_.getRawDataMutex());

  // Loop over all layers that should be set
  for (auto sourceLayerIterator = sourceMap.getLayers().begin(); sourceLayerIterator != sourceMap.getLayers().end();
       sourceLayerIterator++) {
    // skip "mask" layer
    if (*sourceLayerIterator == parameters_.maskedReplaceServiceMaskLayerName_) {
      continue;
    }
    grid_map::Matrix& sourceLayer = sourceMap[*sourceLayerIterator];
    // Check if the layer exists in the elevation map
    if (map_.getRawGridMap().exists(*sourceLayerIterator)) {
      grid_map::Matrix& destinationLayer = map_.getRawGridMap()[*sourceLayerIterator];
      for (grid_map::GridMapIterator destinationIterator(map_.getRawGridMap()); !destinationIterator.isPastEnd(); ++destinationIterator) {
        // Use the position to find corresponding indices in source and destination
        const grid_map::Index destinationIndex(*destinationIterator);
        grid_map::Position position;
        map_.getRawGridMap().getPosition(*destinationIterator, position);

        if (!sourceMap.isInside(position)) {
          continue;
        }

        grid_map::Index sourceIndex;
        sourceMap.getIndex(position, sourceIndex);
        // If the mask allows it, set the value from source to destination
        if (!std::isnan(mask(sourceIndex(0), sourceIndex(1)))) {
          destinationLayer(destinationIndex(0), destinationIndex(1)) = sourceLayer(sourceIndex(0), sourceIndex(1));
        }
      }
    } else {
      ROS_ERROR("Masked replace service: Layer %s does not exist!", sourceLayerIterator->c_str());
    }
  }

  return true;
}

bool ElevationMapping::saveMapServiceCallback(grid_map_msgs::ProcessFile::Request& request,
                                              grid_map_msgs::ProcessFile::Response& response) {
  ROS_INFO("Saving map to file.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  std::string topic = nodeHandle_.getNamespace() + "/elevation_map";
  if (!request.topic_name.empty()) {
    topic = nodeHandle_.getNamespace() + "/" + request.topic_name;
  }
  response.success = static_cast<unsigned char>(grid_map::GridMapRosConverter::saveToBag(map_.getFusedGridMap(), request.file_path, topic));
  response.success = static_cast<unsigned char>(
      (grid_map::GridMapRosConverter::saveToBag(map_.getRawGridMap(), request.file_path + "_raw", topic + "_raw")) &&
      static_cast<bool>(response.success));
  return static_cast<bool>(response.success);
}

bool ElevationMapping::loadMapServiceCallback(grid_map_msgs::ProcessFile::Request& request,
                                              grid_map_msgs::ProcessFile::Response& response) {
  ROS_WARN("Loading from bag file.");
  boost::recursive_mutex::scoped_lock scopedLockFused(map_.getFusedDataMutex());
  boost::recursive_mutex::scoped_lock scopedLockRaw(map_.getRawDataMutex());

  std::string topic = nodeHandle_.getNamespace();
  if (!request.topic_name.empty()) {
    topic += "/" + request.topic_name;
  } else {
    topic += "/elevation_map";
  }

  response.success =
      static_cast<unsigned char>(grid_map::GridMapRosConverter::loadFromBag(request.file_path, topic, map_.getFusedGridMap()));
  response.success = static_cast<unsigned char>(
      grid_map::GridMapRosConverter::loadFromBag(request.file_path + "_raw", topic + "_raw", map_.getRawGridMap()) &&
      static_cast<bool>(response.success));

  // Update timestamp for visualization in ROS
  map_.setTimestamp(ros::Time::now());
  map_.postprocessAndPublishRawElevationMap();
  return static_cast<bool>(response.success);
}

void ElevationMapping::resetMapUpdateTimer() {
  mapUpdateTimer_.stop();
  ros::Duration periodSinceLastUpdate = ros::Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > parameters_.maxNoUpdateDuration_) {
    periodSinceLastUpdate.fromSec(0.0);
  }
  mapUpdateTimer_.setPeriod(parameters_.maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer() {
  mapUpdateTimer_.stop();
}

}  // namespace elevation_mapping
