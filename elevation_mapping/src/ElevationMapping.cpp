/*
 * ElevationMapping.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */
#include "elevation_mapping/ElevationMapping.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/sensor_processors/StructuredLightSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>

// PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

// Kindr
#include <kindr/Core>
#include <kindr_ros/kindr_ros.hpp>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>
#include <math.h>
#include <limits>

using namespace std;
using namespace grid_map;
using namespace ros;
using namespace tf;
using namespace pcl;
using namespace kindr;
using namespace kindr_ros;

namespace elevation_mapping {

ElevationMapping::ElevationMapping(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      map_(nodeHandle),
      robotMotionMapUpdater_(nodeHandle),
      isContinouslyFusing_(false),
      ignoreRobotMotionUpdates_(false)
{
  ROS_INFO("Elevation mapping node started.");

  readParameters();
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &ElevationMapping::pointCloudCallback, this);
  if (!robotPoseTopic_.empty()) {
    robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_, 1);
    robotPoseCache_.connectInput(robotPoseSubscriber_);
    robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  } else {
    ignoreRobotMotionUpdates_ = true;
  }

  mapUpdateTimer_ = nodeHandle_.createTimer(maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);

  // Multi-threading for fusion.
  AdvertiseServiceOptions advertiseServiceOptionsForTriggerFusion = AdvertiseServiceOptions::create<std_srvs::Empty>(
      "trigger_fusion", boost::bind(&ElevationMapping::fuseEntireMap, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  fusionTriggerService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForTriggerFusion);

  AdvertiseServiceOptions advertiseServiceOptionsForGetSubmap = AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_submap", boost::bind(&ElevationMapping::getSubmap, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  submapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetSubmap);

  AdvertiseServiceOptions advertiseServiceOptionsForGetRawSubmap = AdvertiseServiceOptions::create<grid_map_msgs::GetGridMap>(
      "get_raw_submap", boost::bind(&ElevationMapping::getRawSubmap, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  rawSubmapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetRawSubmap);

  if (!fusedMapPublishTimerDuration_.isZero()) {
    TimerOptions timerOptions = TimerOptions(
        fusedMapPublishTimerDuration_,
        boost::bind(&ElevationMapping::publishFusedMapCallback, this, _1), &fusionServiceQueue_,
        false, false);
    fusedMapPublishTimer_ = nodeHandle_.createTimer(timerOptions);
  }

  // Multi-threading for visibility cleanup.
  if (map_.enableVisibilityCleanup_ && !visibilityCleanupTimerDuration_.isZero()){
    TimerOptions timerOptions = TimerOptions(
        visibilityCleanupTimerDuration_,
        boost::bind(&ElevationMapping::visibilityCleanupCallback, this, _1), &visibilityCleanupQueue_,
        false, false);
    visibilityCleanupTimer_ = nodeHandle_.createTimer(timerOptions);
  }

  clearMapService_ = nodeHandle_.advertiseService("clear_map", &ElevationMapping::clearMap, this);
  saveMapService_ = nodeHandle_.advertiseService("save_map", &ElevationMapping::saveMap, this);

  initialize();
}

ElevationMapping::~ElevationMapping()
{
  fusionServiceQueue_.clear();
  fusionServiceQueue_.disable();
  nodeHandle_.shutdown();
  fusionServiceThread_.join();
}

bool ElevationMapping::readParameters()
{
  // ElevationMapping parameters.
  nodeHandle_.param("point_cloud_topic", pointCloudTopic_, string("/points"));
  nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string("/pose"));
  nodeHandle_.param("track_point_frame_id", trackPointFrameId_, string("/robot"));
  nodeHandle_.param("track_point_x", trackPoint_.x(), 0.0);
  nodeHandle_.param("track_point_y", trackPoint_.y(), 0.0);
  nodeHandle_.param("track_point_z", trackPoint_.z(), 0.0);

  nodeHandle_.param("robot_pose_cache_size", robotPoseCacheSize_, 200);
  ROS_ASSERT(robotPoseCacheSize_ >= 0);

  double minUpdateRate;
  nodeHandle_.param("min_update_rate", minUpdateRate, 2.0);
  if (minUpdateRate == 0.0) {
    maxNoUpdateDuration_.fromSec(0.0);
    ROS_WARN("Rate for publishing the map is zero.");
  } else {
    maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);
  }
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());

  double timeTolerance;
  nodeHandle_.param("time_tolerance", timeTolerance, 0.0);
  timeTolerance_.fromSec(timeTolerance);

  double fusedMapPublishingRate;
  nodeHandle_.param("fused_map_publishing_rate", fusedMapPublishingRate, 1.0);
  if (fusedMapPublishingRate == 0.0) {
    fusedMapPublishTimerDuration_.fromSec(0.0);
    ROS_WARN("Rate for publishing the fused map is zero. The fused elevation map will not be published unless the service `triggerFusion` is called.");
  } else if (std::isinf(fusedMapPublishingRate)){
    isContinouslyFusing_ = true;
    fusedMapPublishTimerDuration_.fromSec(0.0);
  } else {
    fusedMapPublishTimerDuration_.fromSec(1.0 / fusedMapPublishingRate);
  }

  double visibilityCleanupRate;
  nodeHandle_.param("visibility_cleanup_rate", visibilityCleanupRate, 1.0);
  if (visibilityCleanupRate == 0.0) {
    visibilityCleanupTimerDuration_.fromSec(0.0);
    ROS_WARN("Rate for visibility cleanup is zero and therefore disabled.");
  }
  else {
    visibilityCleanupTimerDuration_.fromSec(1.0 / visibilityCleanupRate);
    map_.visibilityCleanupDuration_ = 1.0 / visibilityCleanupRate;
  }


  // ElevationMap parameters. TODO Move this to the elevation map class.
  string frameId;
  nodeHandle_.param("map_frame_id", frameId, string("/map"));
  map_.setFrameId(frameId);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position.x(), 0.0);
  nodeHandle_.param("position_y", position.y(), 0.0);
  nodeHandle_.param("resolution", resolution, 0.01);
  map_.setGeometry(length, resolution, position);

  nodeHandle_.param("min_variance", map_.minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", map_.maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", map_.mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", map_.multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", map_.minHorizontalVariance_, pow(resolution / 2.0, 2)); // two-sigma
  nodeHandle_.param("max_horizontal_variance", map_.maxHorizontalVariance_, 0.5);
  nodeHandle_.param("underlying_map_topic", map_.underlyingMapTopic_, string());
  nodeHandle_.param("enable_visibility_cleanup", map_.enableVisibilityCleanup_, true);
  nodeHandle_.param("scanning_duration", map_.scanningDuration_, 1.0);

  // SensorProcessor parameters.
  string sensorType;
  nodeHandle_.param("sensor_processor/type", sensorType, string("structured_light"));
  if (sensorType == "structured_light") {
    sensorProcessor_.reset(new StructuredLightSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "perfect") {
    sensorProcessor_.reset(new PerfectSensorProcessor(nodeHandle_, transformListener_));
  } else {
    ROS_ERROR("The sensor type %s is not available.", sensorType.c_str());
  }
  if (!sensorProcessor_->readParameters()) return false;
  if (!robotMotionMapUpdater_.readParameters()) return false;

  return true;
}

bool ElevationMapping::initialize()
{
  ROS_INFO("Elevation mapping node initializing ... ");
  fusionServiceThread_ = boost::thread(boost::bind(&ElevationMapping::runFusionServiceThread, this));
  Duration(1.0).sleep(); // Need this to get the TF caches fill up.
  resetMapUpdateTimer();
  fusedMapPublishTimer_.start();
  visibilityCleanupThread_ = boost::thread(boost::bind(&ElevationMapping::visibilityCleanupThread, this));
  visibilityCleanupTimer_.start();
  ROS_INFO("Done.");
  return true;
}

void ElevationMapping::runFusionServiceThread()
{
  static const double timeout = 0.05;

  while (nodeHandle_.ok()) {
    fusionServiceQueue_.callAvailable(ros::WallDuration(timeout));
  }
}

void ElevationMapping::visibilityCleanupThread()
{
  static const double timeout = 0.05;

  while (nodeHandle_.ok()) {
    visibilityCleanupQueue_.callAvailable(ros::WallDuration(timeout));
  }
}

void ElevationMapping::pointCloudCallback(
    const sensor_msgs::PointCloud2& rawPointCloud)
{
  stopMapUpdateTimer();

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud.
  // TODO Double check with http://wiki.ros.org/hydro/Migration
  pcl::PCLPointCloud2 pcl_pc;
  pcl_conversions::toPCL(rawPointCloud, pcl_pc);

  PointCloud<PointXYZRGB>::Ptr pointCloud(new PointCloud<PointXYZRGB>);
  pcl::fromPCLPointCloud2(pcl_pc, *pointCloud);
  lastPointCloudUpdateTime_.fromNSec(1000 * pointCloud->header.stamp);

  ROS_INFO("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(lastPointCloudUpdateTime_)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Get robot pose covariance matrix at timestamp of point cloud.
  Eigen::Matrix<double, 6, 6> robotPoseCovariance;
  robotPoseCovariance.setZero();
  if (!ignoreRobotMotionUpdates_) {
    boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(lastPointCloudUpdateTime_);
    if (!poseMessage) {
      ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", lastPointCloudUpdateTime_.toSec());
      return;
    }
    robotPoseCovariance = Eigen::Map<const Eigen::MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);
  }

  // Process point cloud.
  PointCloud<PointXYZRGB>::Ptr pointCloudProcessed(new PointCloud<PointXYZRGB>);
  Eigen::VectorXf measurementVariances;
  if (!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed,
                                 measurementVariances)) {
    ROS_ERROR("Point cloud could not be processed.");
    resetMapUpdateTimer();
    return;
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances, lastPointCloudUpdateTime_, Eigen::Affine3d(sensorProcessor_->transformationSensorToMap_))) {
    ROS_ERROR("Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_.publishRawElevationMap();
  if (isContinouslyFusing_ && map_.hasFusedMapSubscribers()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback(const ros::TimerEvent&)
{
  ROS_WARN("Elevation map is updated without data from the sensor.");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  stopMapUpdateTimer();
  ros::Time time = ros::Time::now();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish elevation map.
  map_.publishRawElevationMap();
  if (isContinouslyFusing_ && map_.hasFusedMapSubscribers()) {
    map_.fuseAll();
    map_.publishFusedElevationMap();
  }

  resetMapUpdateTimer();
}

void ElevationMapping::publishFusedMapCallback(const ros::TimerEvent&)
{
  if (!map_.hasFusedMapSubscribers()) return;
  ROS_DEBUG("Elevation map is fused and published from timer.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
}

void ElevationMapping::visibilityCleanupCallback(const ros::TimerEvent&)
{
  ROS_DEBUG("Elevation map is running visibility cleanup.");
  // Copy constructors for thread-safety.
  map_.visibilityCleanup(ros::Time(lastPointCloudUpdateTime_));
}

bool ElevationMapping::fuseEntireMap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  map_.publishFusedElevationMap();
  return true;
}

bool ElevationMapping::updatePrediction(const ros::Time& time)
{
  if (ignoreRobotMotionUpdates_) return true;

  ROS_DEBUG("Updating map with latest prediction from time %f.", robotPoseCache_.getLatestTime().toSec());

  if (time + timeTolerance_ < map_.getTimeOfLastUpdate()) {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return false;
  } else if (time < map_.getTimeOfLastUpdate()) {
    ROS_DEBUG("Requested update with time stamp %f, but time of last update was %f. Ignoring update.", time.toSec(), map_.getTimeOfLastUpdate().toSec());
    return true;
  }

  // Get robot pose at requested time.
  boost::shared_ptr<geometry_msgs::PoseWithCovarianceStamped const> poseMessage = robotPoseCache_.getElemBeforeTime(time);
  if (!poseMessage) {
    ROS_ERROR("Could not get pose information from robot for time %f. Buffer empty?", time.toSec());
    return false;
  }

  HomTransformQuatD robotPose;
  convertFromRosGeometryMsg(poseMessage->pose.pose, robotPose);
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Eigen::Matrix<double, 6, 6> robotPoseCovariance = Eigen::Map<
      const Eigen::Matrix<double, 6, 6, Eigen::RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
}

bool ElevationMapping::updateMapLocation()
{
  ROS_DEBUG("Elevation map is checked for relocalization.");

  geometry_msgs::PointStamped trackPoint;
  trackPoint.header.frame_id = trackPointFrameId_;
  trackPoint.header.stamp = ros::Time(0);
  convertToRosGeometryMsg(trackPoint_, trackPoint.point);
  geometry_msgs::PointStamped trackPointTransformed;

  try {
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
  } catch (TransformException &ex) {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Position3D position3d;
  convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  grid_map::Position position = position3d.vector().head(2);
  map_.move(position);
  return true;
}

bool ElevationMapping::getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength);

  bool isSuccess;
  Index index;
  GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    vector<string> layers;
    for (const auto& layer : request.layers) {
      layers.push_back(layer);
    }
    GridMapRosConverter::toMessage(subMap, layers, response.map);
  }

  ROS_DEBUG("Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());
  return isSuccess;
}

bool ElevationMapping::getRawSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response)
{
  grid_map::Position requestedSubmapPosition(request.position_x, request.position_y);
  Length requestedSubmapLength(request.length_x, request.length_y);
  ROS_DEBUG("Elevation raw submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));
  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  bool isSuccess;
  Index index;
  GridMap subMap = map_.getRawGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.layers.empty()) {
    GridMapRosConverter::toMessage(subMap, response.map);
  } else {
    vector<string> layers;
    for (const auto& layer : request.layers) {
      layers.push_back(layer);
    }
    GridMapRosConverter::toMessage(subMap, layers, response.map);
  }
  return isSuccess;
}

bool ElevationMapping::clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Clearing map.");
  return map_.clear();
}

bool ElevationMapping::saveMap(grid_map_msgs::ProcessFile::Request& request, grid_map_msgs::ProcessFile::Response& response)
{
  ROS_INFO("Saving map to file.");
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll();
  std::string topic = nodeHandle_.getNamespace() + "/elevation_map";
  if (!request.topic_name.empty()) {
    topic = nodeHandle_.getNamespace() + "/" + request.topic_name;
  }
  response.success = GridMapRosConverter::saveToBag(map_.getFusedGridMap(), request.file_path, topic);
  response.success = GridMapRosConverter::saveToBag(map_.getRawGridMap(), request.file_path + "_raw", topic + "_raw");
  return response.success;
}

void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  Duration periodSinceLastUpdate = ros::Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) periodSinceLastUpdate.fromSec(0.0);
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace */
