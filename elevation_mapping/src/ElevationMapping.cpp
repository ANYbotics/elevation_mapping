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
#include "elevation_mapping/sensor_processors/KinectSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/StereoSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/LaserSensorProcessor.hpp"
#include "elevation_mapping/sensor_processors/PerfectSensorProcessor.hpp"

// Grid Map
#include <grid_map_msg/GridMap.h>

//PCL
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

// Kindr
#include <kindr/poses/PoseEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>
#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/thirdparty/ros/RosEigen.hpp>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/recursive_mutex.hpp>

// STL
#include <string>

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
      map_(nodeHandle),
      robotMotionMapUpdater_(nodeHandle)
{
  ROS_INFO("Elevation mapping node started.");

  readParameters();
  pointCloudSubscriber_ = nodeHandle_.subscribe(pointCloudTopic_, 1, &ElevationMapping::pointCloudCallback, this);
  robotPoseSubscriber_.subscribe(nodeHandle_, robotPoseTopic_, 1);
  robotPoseCache_.connectInput(robotPoseSubscriber_);
  robotPoseCache_.setCacheSize(robotPoseCacheSize_);
  mapUpdateTimer_ = nodeHandle_.createTimer(maxNoUpdateDuration_, &ElevationMapping::mapUpdateTimerCallback, this, true, false);

  // Multi-threading for fusion.
  AdvertiseServiceOptions advertiseServiceOptionsForTriggerFusion = AdvertiseServiceOptions::create<std_srvs::Empty>(
      "trigger_fusion", boost::bind(&ElevationMapping::fuseEntireMap, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  fusionTriggerService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForTriggerFusion);

  AdvertiseServiceOptions advertiseServiceOptionsForGetSubmap = AdvertiseServiceOptions::create<grid_map_msg::GetGridMap>(
      "get_grid_map", boost::bind(&ElevationMapping::getSubmap, this, _1, _2), ros::VoidConstPtr(),
      &fusionServiceQueue_);
  submapService_ = nodeHandle_.advertiseService(advertiseServiceOptionsForGetSubmap);

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
  nodeHandle_.param("robot_pose_with_covariance_topic", robotPoseTopic_, string("/robot_state/pose"));
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

  // ElevationMap parameters. TODO Move this to the elevation map class.
  string frameId;
  nodeHandle_.param("map_frame_id", frameId, string("/map"));
  map_.setFrameId(frameId);

  Eigen::Array2d length;
  Eigen::Vector2d position;
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
  nodeHandle_.param("surface_normal_estimation_radius", map_.surfaceNormalEstimationRadius_, 0.05);

  string surfaceNormalPositiveAxis;
  nodeHandle_.param("surface_normal_positive_axis", surfaceNormalPositiveAxis, string("z"));
  if (surfaceNormalPositiveAxis == "z") {
    map_.surfaceNormalPositiveAxis_ = Vector3d::UnitZ();
  } else if (surfaceNormalPositiveAxis == "y") {
    map_.surfaceNormalPositiveAxis_ = Vector3d::UnitY();
  } else if (surfaceNormalPositiveAxis == "x") {
    map_.surfaceNormalPositiveAxis_ = Vector3d::UnitX();
  } else {
    ROS_ERROR("The surface normal positive axis '%s' is not valid.", surfaceNormalPositiveAxis.c_str());
  }

  // SensorProcessor parameters.
  string sensorType;
  nodeHandle_.param("sensor_processor/type", sensorType, string("Kinect"));
  if (sensorType == "Kinect") {
    sensorProcessor_.reset(new KinectSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "Stereo") {
    sensorProcessor_.reset(new StereoSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "Laser") {
    sensorProcessor_.reset(new LaserSensorProcessor(nodeHandle_, transformListener_));
  } else if (sensorType == "Perfect") {
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
  ROS_INFO("Done.");
  return true;
}

void ElevationMapping::runFusionServiceThread()
{
  static const double timeout = 0.01;

  while(nodeHandle_.ok())
  {
    fusionServiceQueue_.callAvailable(ros::WallDuration(timeout));
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
  Time time;
  time.fromNSec(1000 * pointCloud->header.stamp);

  ROS_DEBUG("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  // Update map location.
  updateMapLocation();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    ROS_ERROR("Updating process noise failed.");
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
  if(!sensorProcessor_->process(pointCloud, robotPoseCovariance, pointCloudProcessed, measurementVariances))
  {
    ROS_ERROR("Point cloud could not be processed.");
    resetMapUpdateTimer();
    return;
  }

  // Add point cloud to elevation map.
  if (!map_.add(pointCloudProcessed, measurementVariances))
  {
    ROS_ERROR("Adding point cloud to elevation map failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish raw elevation map.
  if (!map_.publishRawElevationMap()) ROS_DEBUG("Elevation map has not been broadcasted.");

  // TODO Add option for continous fusion.
//  boost::recursive_mutex::scoped_lock scopedLock2(map_.getFusedDataMutex());
//  map_.fuseAll(true);
//  map_.publishElevationMap();

  resetMapUpdateTimer();
}

void ElevationMapping::mapUpdateTimerCallback(const ros::TimerEvent&)
{
  ROS_WARN("Elevation map is updated without data from the sensor.");

  boost::recursive_mutex::scoped_lock scopedLock(map_.getRawDataMutex());

  stopMapUpdateTimer();
  Time time = Time::now();

  // Update map from motion prediction.
  if (!updatePrediction(time)) {
    ROS_ERROR("Updating process noise failed.");
    resetMapUpdateTimer();
    return;
  }

  // Publish raw elevation map.
  if (!map_.publishRawElevationMap()) ROS_DEBUG("Elevation map has not been broadcasted.");

  resetMapUpdateTimer();
}

bool ElevationMapping::fuseEntireMap(std_srvs::Empty::Request&, std_srvs::Empty::Response&)
{
  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseAll(true);
  map_.publishElevationMap();
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
  // Covariance is stored in row-major in ROS: http://docs.ros.org/api/geometry_msgs/html/msg/PoseWithCovariance.html
  Matrix<double, 6, 6> robotPoseCovariance = Map<const Matrix<double, 6, 6, RowMajor>>(poseMessage->pose.covariance.data(), 6, 6);

  // Compute map variance update from motion prediction.
  robotMotionMapUpdater_.update(map_, robotPose, robotPoseCovariance, time);

  return true;
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
    transformListener_.transformPoint(map_.getFrameId(), trackPoint, trackPointTransformed);
  }
  catch (TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }

  Position3D position3d;
  convertFromRosGeometryMsg(trackPointTransformed.point, position3d);
  Vector2d position = position3d.vector().head(2);
  map_.move(position);
  return true;
}

bool ElevationMapping::getSubmap(grid_map_msg::GetGridMap::Request& request, grid_map_msg::GetGridMap::Response& response)
{
  Vector2d requestedSubmapPosition(request.positionX, request.positionY);
  Array2d requestedSubmapLength(request.lengthX, request.lengthY);
  ROS_DEBUG("Elevation submap request: Position x=%f, y=%f, Length x=%f, y=%f.", requestedSubmapPosition.x(), requestedSubmapPosition.y(), requestedSubmapLength(0), requestedSubmapLength(1));

  bool computeSurfaceNormals = false;
  if (request.dataDefinition.empty()) {
    computeSurfaceNormals = true;
  } else {
    for (const auto& type : request.dataDefinition) {
      if (type.data.find("surface_normal") != std::string::npos) computeSurfaceNormals = true;
    }
  }

  boost::recursive_mutex::scoped_lock scopedLock(map_.getFusedDataMutex());
  map_.fuseArea(requestedSubmapPosition, requestedSubmapLength, computeSurfaceNormals);

  bool isSuccess;
  Array2i index;
  grid_map::GridMap subMap = map_.getFusedGridMap().getSubmap(requestedSubmapPosition, requestedSubmapLength, index, isSuccess);
  scopedLock.unlock();

  if (request.dataDefinition.empty()) {
    subMap.toMessage(response.gridMap);
  } else {
    vector<string> types;
    for (const auto& type : request.dataDefinition) {
      types.push_back(type.data);
    }
    subMap.toMessage(types, response.gridMap);
  }

  ROS_DEBUG("Elevation submap responded with timestamp %f.", map_.getTimeOfLastFusion().toSec());
  return isSuccess;
}

void ElevationMapping::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  Duration periodSinceLastUpdate = Time::now() - map_.getTimeOfLastUpdate();
  if (periodSinceLastUpdate > maxNoUpdateDuration_) periodSinceLastUpdate.fromSec(0.0);
  mapUpdateTimer_.setPeriod(maxNoUpdateDuration_ - periodSinceLastUpdate);
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace */
