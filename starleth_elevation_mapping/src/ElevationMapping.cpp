/*
 * ElevationMap.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "ElevationMapping.hpp"

// StarlETH Navigation
#include "ElevationMappingHelpers.hpp"
#include <ElevationMessageHelpers.hpp>
#include <starleth_elevation_msg/ElevationMap.h>
#include <EigenConversions.hpp>
#include <TransformationMath.hpp>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

// ROS
#include <tf_conversions/tf_eigen.h>

// Math
#include <math.h>

using namespace std;
using namespace Eigen;
using namespace pcl;
using namespace ros;
using namespace tf;

namespace starleth_elevation_mapping {

// TODO Split this class into classes: ElevationMap (ROS-independent), KinectSensor, robotPredictionModel

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
  nodeHandle.param("sensor_cutoff_min_depth", sensorCutoffMaxDepth_, 0.2);
  nodeHandle.param("sensor_cutoff_max_depth", sensorCutoffMaxDepth_, 2.0);
  nodeHandle.param("sensor_model_factor_a", sensorModelFactorA_, 0.003);
  nodeHandle.param("sensor_model_factor_b", sensorModelFactorB_, 0.015);
  nodeHandle.param("sensor_model_factor_c", sensorModelFactorC_, 0.25);
  nodeHandle.param("length_in_x", length_(0), 1.5);
  nodeHandle.param("length_in_y", length_(1), 1.5);
  nodeHandle.param("resolution", resolution_, 0.1);
  nodeHandle.param("min_variance", minVariance_, pow(0.003, 2));
  nodeHandle.param("max_variance", maxVariance_, pow(0.03, 2));
  nodeHandle.param("mahalanobis_distance_threshold", mahalanobisDistanceThreshold_, 2.0);
  nodeHandle.param("multi_height_noise", multiHeightNoise_, pow(0.003, 2));
  nodeHandle.param("bigger_height_threshold_factor", biggerHeightThresholdFactor_, 4.0);
  nodeHandle.param("bigger_height_noise_factor", biggerHeightNoiseFactor_, 2.0);

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
  elevationMapToParentTransform_.setIdentity();
  robotPoseCovariance_.setZero();
  resizeMap(parameters_.length_);
  resetMap();
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
  cleanPointCloud(pointCloud);
  VectorXf measurementDistances;
  getMeasurementDistances(pointCloud, measurementDistances);
  if (!transformPointCloud(pointCloud, parameters_.elevationMapFrameId_)) ROS_ERROR("ElevationMap: Point cloud transform failed for time stamp %f.", time.toSec());
  else if (!addToElevationMap(pointCloud, measurementDistances)) ROS_ERROR("ElevationMap: Adding point cloud to elevation map failed.");
  cleanElevationMap();
  generateFusedMap();
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
  cleanElevationMap();
  generateFusedMap();
  if (!publishElevationMap()) ROS_ERROR("ElevationMap: Elevation map has not been broadcasted.");
  resetMapUpdateTimer();
}

bool ElevationMapping::broadcastElevationMapTransform(const ros::Time& time)
{
  tf::Transform tfTransform;
  poseEigenToTF(elevationMapToParentTransform_, tfTransform);
  transformBroadcaster_.sendTransform(tf::StampedTransform(tfTransform, time, parameters_.parentFrameId_, parameters_.elevationMapFrameId_));
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
  Matrix<double, 6, 6> newRobotPoseCovariance = Map<const MatrixXd>(poseMessage->pose.covariance.data(), 6, 6);

  Vector3d positionVariance = robotPoseCovariance_.diagonal().head(3);
  Vector3d newPositionVariance = newRobotPoseCovariance.diagonal().head(3);

  Vector3f variancePrediction = (newPositionVariance - positionVariance).cast<float>();

  // Update map
  varianceData_ = (varianceData_.array() + variancePrediction.z()).matrix();
  horizontalVarianceDataX_ = (horizontalVarianceDataX_.array() + variancePrediction.x()).matrix();
  horizontalVarianceDataY_ = (horizontalVarianceDataY_.array() + variancePrediction.y()).matrix();

  // Make valid variances
  varianceData_ = varianceData_.cwiseMax(0.0);
  horizontalVarianceDataX_ = horizontalVarianceDataX_.cwiseMax(0.0);
  horizontalVarianceDataY_ = horizontalVarianceDataY_.cwiseMax(0.0);

  robotPoseCovariance_ = newRobotPoseCovariance;

  return true;
}

bool ElevationMapping::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
  PassThrough<PointXYZRGB> passThroughFilter;
  PointCloud<PointXYZRGB> tempPointCloud;

  passThroughFilter.setInputCloud(pointCloud);
  passThroughFilter.setFilterFieldName("z");
  passThroughFilter.setFilterLimits(parameters_.sensorCutoffMinDepth_, parameters_.sensorCutoffMaxDepth_);
  // This makes the point cloud also dense (no NaN points).
  passThroughFilter.filter(tempPointCloud);
  tempPointCloud.is_dense = true;
  pointCloud->swap(tempPointCloud);

  ROS_DEBUG("ElevationMap: cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
  return true;
}

bool ElevationMapping::getMeasurementDistances(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances)
{
  // TODO Measurement distances should be added to the point cloud.
  measurementDistances.resize(pointCloud->size());

  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];
    measurementDistances[i] = Vector3f(point.x, point.y, point.z).norm();
  }

  return true;
}

bool ElevationMapping::transformPointCloud(
    const PointCloud<PointXYZRGB>::Ptr pointCloud,
    const std::string& targetFrame)
{
  StampedTransform transformTf;
  string sourceFrame = pointCloud->header.frame_id;
  Time timeStamp =  pointCloud->header.stamp;

  PointCloud<PointXYZRGB>::Ptr pointCloudTransformed(new PointCloud<PointXYZRGB>);

  try
  {
    transformListener_.waitForTransform(targetFrame, sourceFrame, timeStamp, ros::Duration(parameters_.maxNoUpdateDuration_));
    transformListener_.lookupTransform(targetFrame, sourceFrame, timeStamp, transformTf);
    Affine3d transform;
    poseTFToEigen(transformTf, transform);
    pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
    pointCloud->swap(*pointCloudTransformed);
    pointCloud->header.frame_id = targetFrame;
//    pointCloud->header.stamp = timeStamp;
    ROS_DEBUG("ElevationMap: Point cloud transformed for time stamp %f.", timeStamp.toSec());
    return true;
  }
  catch (TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool ElevationMapping::addToElevationMap(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances)
{
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];

    Array2i index;
    if (!starleth_elevation_msg::getIndexFromPosition(
        index, Vector2d(point.x, point.y), parameters_.length_, parameters_.resolution_, getMapBufferSize(), circularBufferStartIndex_))
      continue; // Skip this point if it does not lie within the elevation map

    auto& elevation = elevationData_(index(0), index(1));
    auto& variance = varianceData_(index(0), index(1));
    auto& horizontalVarianceX = horizontalVarianceDataX_(index(0), index(1));
    auto& horizontalVarianceY = horizontalVarianceDataY_(index(0), index(1));
    auto& color = colorData_(index(0), index(1));
    auto& multiHeightLabel = labelData_(index(0), index(1));
    auto& measurementDistance = measurementDistances[i];

    float measurementStandardDeviation = parameters_.sensorModelFactorA_
        + parameters_.sensorModelFactorB_ * pow(measurementDistance - parameters_.sensorModelFactorC_, 2);
    float measurementVariance = pow(measurementStandardDeviation, 2);

    if (std::isnan(elevation) || std::isinf(variance))
    {
      // No prior information in elevation map, use measurement.
      elevation = point.z;
      variance = measurementVariance;
      starleth_elevation_msg::copyColorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    double mahalanobisDistance = sqrt(pow(point.z - elevation, 2) / variance);

    if (mahalanobisDistance < parameters_.mahalanobisDistanceThreshold_)
    {
      // Fuse measurement with elevation map data.
      elevation = (variance * point.z + measurementVariance * elevation) / (variance + measurementVariance);
      variance =  (measurementVariance * variance) / (measurementVariance + variance);
      // TODO add color fusion
      starleth_elevation_msg::copyColorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }
    if (point.z > elevation && mahalanobisDistance < parameters_.biggerHeightThresholdFactor_ * parameters_.mahalanobisDistanceThreshold_)
    {
      // Overwrite elevation map information with measurement data
      elevation = point.z;
      variance = measurementVariance;
      starleth_elevation_msg::copyColorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    if (point.z > elevation && mahalanobisDistance > parameters_.biggerHeightThresholdFactor_ * parameters_.mahalanobisDistanceThreshold_)
    {
      variance += parameters_.biggerHeightNoiseFactor_ * parameters_.multiHeightNoise_;
      continue;
    }

    horizontalVarianceX = 0.0;
    horizontalVarianceY = 0.0;

    // TODO Add label to cells which are potentially multi-level

    // Add noise to cells which have ignored lower values,
    // such we outliers and moving objects are removed
    variance += parameters_.multiHeightNoise_;
  }

  return true;
}

bool ElevationMapping::cleanElevationMap()
{
  varianceData_ = varianceData_.unaryExpr(VarianceClampOperator<double>(parameters_.minVariance_, parameters_.maxVariance_));
  return true;
}

bool ElevationMapping::publishElevationMap()
{
  if (elevationMapPublisher_.getNumSubscribers() < 1) return false;

  starleth_elevation_msg::ElevationMap elevationMapMessage;

  elevationMapMessage.header.stamp = timeOfLastUpdate_;
  elevationMapMessage.header.frame_id = parameters_.elevationMapFrameId_;
  elevationMapMessage.resolution = parameters_.resolution_;
  elevationMapMessage.lengthInX = parameters_.length_(0);
  elevationMapMessage.lengthInY = parameters_.length_(1);
  elevationMapMessage.outerStartIndex = circularBufferStartIndex_(0);
  elevationMapMessage.innerStartIndex = circularBufferStartIndex_(1);

  starleth_elevation_msg::matrixEigenToMultiArrayMessage(elevationData_, elevationMapMessage.elevation);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(varianceData_, elevationMapMessage.variance);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(colorData_, elevationMapMessage.color);

  elevationMapPublisher_.publish(elevationMapMessage);

  ROS_DEBUG("Elevation map has been published.");

  return true;
}

bool ElevationMapping::resizeMap(const Eigen::Array2d& length)
{
  int nRows = static_cast<int>(round(length(0) / parameters_.resolution_));
  int nCols = static_cast<int>(round(length(1) / parameters_.resolution_));
  parameters_.length_ = (Array2i(nRows, nCols).cast<double>() * parameters_.resolution_).matrix();

  elevationData_.resize(nRows, nCols);
  varianceData_.resize(nRows, nCols);
  horizontalVarianceDataX_.resize(nRows, nCols);
  horizontalVarianceDataY_.resize(nRows, nCols);
  colorData_.resize(nRows, nCols);
  labelData_.resize(nRows, nCols);

  ROS_DEBUG_STREAM("Elevation map matrix resized to " << elevationData_.rows() << " rows and "  << elevationData_.cols() << " columns.");
  return true;
}

Eigen::Vector2i ElevationMapping::getMapBufferSize()
{
  return Vector2i(elevationData_.rows(), elevationData_.cols());
}

bool ElevationMapping::resetMap()
{
  elevationMapToParentTransform_.setIdentity();
  elevationData_.setConstant(NAN);
  varianceData_.setConstant(numeric_limits<float>::infinity());
  horizontalVarianceDataX_.setConstant(numeric_limits<float>::infinity());
  horizontalVarianceDataY_.setConstant(numeric_limits<float>::infinity());
  colorData_.setConstant(0);
  labelData_.setConstant(false);
  circularBufferStartIndex_.setZero();
  return true;
}

void ElevationMapping::resetCols(unsigned int index, unsigned int nCols)
{
  elevationData_.block(index, 0, nCols, getMapBufferSize()[1]).setConstant(NAN);
}

void ElevationMapping::resetRows(unsigned int index, unsigned int nRows)
{
  elevationData_.block(0, index, getMapBufferSize()[0], nRows).setConstant(NAN);
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

  return relocateMap(position);
}

bool ElevationMapping::relocateMap(const Eigen::Vector3d& position)
{
  Vector2d alignedPosition;
  starleth_elevation_msg::getAlignedPosition(position.head(2),
                                        alignedPosition, parameters_.length_,
                                        parameters_.resolution_);

  Vector2d positionShift = alignedPosition - elevationMapToParentTransform_.translation().head(2);

  Array2i indexShift;
  starleth_elevation_msg::getIndexShiftFromPositionShift(indexShift, positionShift, parameters_.resolution_);

  // Delete fields that fall out of map (and become empty cells)
  for (int i = 0; i < indexShift.size(); i++)
  {
    if (indexShift[i] != 0)
    {
      if (abs(indexShift[i]) >= getMapBufferSize()(i))
      {
        // Entire map is dropped
        elevationData_.setConstant(NAN);
      }
      else
      {
        // Drop cells out of map
        int sign = (indexShift[i] > 0 ? 1 : -1);
        int startIndex = circularBufferStartIndex_[i] - (sign < 0 ? 1 : 0);
        int endIndex = startIndex - sign + indexShift[i];
        int nCells = abs(indexShift[i]);

        int index = (sign > 0 ? startIndex : endIndex);
        starleth_elevation_msg::mapIndexWithinRange(index, getMapBufferSize()[i]);

        if (index + nCells <= getMapBufferSize()[i])
        {
          // One region to drop
          if (i == 0) resetCols(index, nCells);
          if (i == 1) resetRows(index, nCells);
        }
        else
        {
          // Two regions to drop
          int firstIndex = index;
          int firstNCells = getMapBufferSize()[i] - firstIndex;
          if (i == 0) resetCols(firstIndex, firstNCells);
          if (i == 1) resetRows(firstIndex, firstNCells);

          int secondIndex = 0;
          int secondNCells = nCells - firstNCells;
          if (i == 0) resetCols(secondIndex, secondNCells);
          if (i == 1) resetRows(secondIndex, secondNCells);
        }
      }
    }
  }

  // Udpate information
  circularBufferStartIndex_ += indexShift;
  starleth_elevation_msg::mapIndexWithinRange(circularBufferStartIndex_, getMapBufferSize());

  elevationMapToParentTransform_.translation().head(2) = alignedPosition;

  return true;
}

bool ElevationMapping::getSubmap(Eigen::MatrixXf& submap, Eigen::Array2i& centerIndex, const Eigen::MatrixXf& map, const Eigen::Vector2d& center, const Eigen::Array2d& size)
{
  Array2i submapTopLeftIndex, submapSize;
  if (!starleth_elevation_msg::getSubmapIndexAndSize(submapTopLeftIndex, centerIndex,
                                                     submapSize, center, size,
                                                     parameters_.length_,
                                                     parameters_.resolution_,
                                                     getMapBufferSize(),
                                                     circularBufferStartIndex_))
  {
    ROS_ERROR("Position of requested submap is not part of the map.");
    return false;
  }

  return getSubmap(submap, map, submapTopLeftIndex, submapSize);
}

bool ElevationMapping::getSubmap(Eigen::MatrixXf& submap, const Eigen::MatrixXf& map, const Eigen::Array2i& topLeftindex, const Eigen::Array2i& size)
{
  std::vector<Eigen::Array2i> submapIndeces;
  std::vector<Eigen::Array2i> submapSizes;

  if(!starleth_elevation_msg::getBufferRegionsForSubmap(submapIndeces, submapSizes, topLeftindex, size, getMapBufferSize(), circularBufferStartIndex_))
  {
     ROS_ERROR("Cannot access submap of this size.");
     return false;
  }

  unsigned int nRows =
      (submapSizes[3](0) + submapSizes[1](0) > submapSizes[2](0) + submapSizes[0](0)) ?
          submapSizes[3](0) + submapSizes[1](0) : submapSizes[2](0) + submapSizes[0](0);
  unsigned int nCols =
      (submapSizes[3](1) + submapSizes[2](1) > submapSizes[1](1) + submapSizes[0](1)) ?
          submapSizes[3](1) + submapSizes[2](1) : submapSizes[1](1) + submapSizes[0](1);

  submap.resize(nRows, nCols);

  submap.topLeftCorner    (submapSizes[0](0), submapSizes[0](1)) = map.block(submapIndeces[0](0), submapIndeces[0](1), submapSizes[0](0), submapSizes[0](1));
  submap.topRightCorner   (submapSizes[1](0), submapSizes[1](1)) = map.block(submapIndeces[1](0), submapIndeces[1](1), submapSizes[1](0), submapSizes[1](1));
  submap.bottomLeftCorner (submapSizes[2](0), submapSizes[2](1)) = map.block(submapIndeces[2](0), submapIndeces[2](1), submapSizes[2](0), submapSizes[2](1));
  submap.bottomRightCorner(submapSizes[3](0), submapSizes[3](1)) = map.block(submapIndeces[3](0), submapIndeces[3](1), submapSizes[3](0), submapSizes[3](1));

  return true;
}

bool ElevationMapping::getSubmap(starleth_elevation_msg::ElevationSubmap::Request& request, starleth_elevation_msg::ElevationSubmap::Response& response)
{
//  res.sum = req.a + req.b;
//  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
//  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

bool ElevationMapping::generateFusedMap()
{
  ROS_DEBUG("Fusing elevation map...");

  // Initializations
  MatrixXf fusedElevationData(elevationData_.rows(), elevationData_.cols());
  MatrixXf fusedVarianceData(varianceData_.rows(), varianceData_.cols());
  fusedElevationData.setConstant(NAN);
  fusedVarianceData.setConstant(numeric_limits<float>::infinity());

  // For each cell in map
  for (unsigned int i = 0; i < fusedElevationData.rows(); ++i)
  {
    for (unsigned int j = 0; j < fusedElevationData.cols(); ++j)
    {
      // Center of submap in map
      Vector2d center;
      starleth_elevation_msg::getPositionFromIndex(center, Array2i(i, j),
                                                   parameters_.length_, parameters_.resolution_,
                                                   getMapBufferSize(), circularBufferStartIndex_);

      // Size of submap (2 sigma bound)
      Array2d size = 4 * Array2d(horizontalVarianceDataX_(i, j), horizontalVarianceDataY_(i, j)).sqrt();

      // TODO Don't skip holes
      if (!(std::isfinite(size[0]) && std::isfinite(size[1]))) continue;

      // Get submap data
      MatrixXf elevationSubmap, variancesSubmap, horizontalVariancesXSubmap, horizontalVariancesYSubmap;
      Array2i centerIndex;
      getSubmap(elevationSubmap, centerIndex, elevationData_, center, size);
      getSubmap(variancesSubmap, centerIndex, varianceData_, center, size);
      getSubmap(horizontalVariancesXSubmap, centerIndex, horizontalVarianceDataX_, center, size);
      getSubmap(horizontalVariancesYSubmap, centerIndex, horizontalVarianceDataY_, center, size);

      // Prepare data fusion
      ArrayXf means, variances, weights;
      Array2i submapBufferSize(elevationSubmap.rows(), elevationSubmap.cols());
      int maxNumberOfCellsToFuse = submapBufferSize.prod();
      means.resize(maxNumberOfCellsToFuse);
      variances.resize(maxNumberOfCellsToFuse);
      weights.resize(maxNumberOfCellsToFuse);

      // Position of center index in submap
      Vector2d centerInSubmap;
      starleth_elevation_msg::getPositionFromIndex(centerInSubmap, centerIndex,
                                                   size, parameters_.resolution_,
                                                   submapBufferSize, Array2i(0, 0));

      unsigned int n = 0;

      for (unsigned int p = 0; p < elevationSubmap.rows(); ++p)
      {
        for (unsigned int q = 0; q < elevationSubmap.cols(); ++q)
        {
          if (!(std::isfinite(elevationSubmap(p, q))
               && std::isfinite(variancesSubmap(p, q))
               && std::isnormal(horizontalVariancesXSubmap(p, q))
               && std::isnormal(horizontalVariancesYSubmap(p, q)))) continue;

          means[n] = elevationSubmap(p, q);
          variances[n] = variancesSubmap(p, q);

          // Compute weight from probability
          Vector2d positionInSubmap;
          starleth_elevation_msg::getPositionFromIndex(positionInSubmap, Array2i(p, q),
                                                       size, parameters_.resolution_,
                                                       submapBufferSize, Array2i(0, 0));

          Vector2d distanceToCenter = positionInSubmap - centerInSubmap;

          float probabilityX = fabs(
                cumulativeDistributionFunction(distanceToCenter.x() - parameters_.resolution_ / 2.0, 0.0, sqrt(horizontalVariancesXSubmap(p, q)))
              - cumulativeDistributionFunction(distanceToCenter.x() + parameters_.resolution_ / 2.0, 0.0, sqrt(horizontalVariancesXSubmap(p, q))));
          float probabilityY = fabs(
                cumulativeDistributionFunction(distanceToCenter.y() - parameters_.resolution_ / 2.0, 0.0, sqrt(horizontalVariancesYSubmap(p, q)))
              - cumulativeDistributionFunction(distanceToCenter.y() + parameters_.resolution_ / 2.0, 0.0, sqrt(horizontalVariancesYSubmap(p, q))));

          weights[n] = probabilityX * probabilityY;

          n++;
        }
      }

      if (n == 0)
      {
        // Nothing to fuse
        fusedElevationData(i, j) = elevationData_(i, j);
        fusedVarianceData(i, j) = varianceData_(i, j);
        continue;
      }

      // Fuse
      means.conservativeResize(n);
      variances.conservativeResize(n);
      weights.conservativeResize(n);

      float mean = (weights * means).sum() / weights.sum();
      float variance = (weights * (variances.square() + means.square())).sum() / weights.sum() - pow(mean, 2);
      if(std::isnan(-variance)) variance = 0.0;

      if (!(std::isfinite(variance) && std::isfinite(mean)))
      {
        ROS_ERROR("Something went wrong when fusing the map: Mean = %f, Variance = %f", mean, variance);
        continue;
      }

      // Add to fused map
      fusedElevationData(i, j) = mean;
      fusedVarianceData(i, j) = variance;
    }
  }

  publishFusedElevationMap(fusedElevationData, fusedVarianceData);

  return true;
}

bool ElevationMapping::publishFusedElevationMap(const Eigen::MatrixXf& fusedElevationData, const Eigen::MatrixXf& fusedVarianceData)
{
  if (fusedElevationMapPublisher_.getNumSubscribers() < 1) return false;

  starleth_elevation_msg::ElevationMap fusedElevationMapMessage;

  fusedElevationMapMessage.header.stamp = timeOfLastUpdate_;
  fusedElevationMapMessage.header.frame_id = parameters_.elevationMapFrameId_;
  fusedElevationMapMessage.resolution = parameters_.resolution_;
  fusedElevationMapMessage.lengthInX = parameters_.length_(0);
  fusedElevationMapMessage.lengthInY = parameters_.length_(1);
  fusedElevationMapMessage.outerStartIndex = circularBufferStartIndex_(0);
  fusedElevationMapMessage.innerStartIndex = circularBufferStartIndex_(1);

  starleth_elevation_msg::matrixEigenToMultiArrayMessage(fusedElevationData, fusedElevationMapMessage.elevation);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(fusedVarianceData, fusedElevationMapMessage.variance);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(colorData_, fusedElevationMapMessage.color);

  fusedElevationMapPublisher_.publish(fusedElevationMapMessage);

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

float ElevationMapping::cumulativeDistributionFunction(float x, float mean, float standardDeviation)
{
  return 0.5 * erfc(-(x-mean)/(standardDeviation*sqrt(2.0)));
}

} /* namespace starleth_elevation_map */
