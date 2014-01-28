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
  robotTwistSubscriber_.subscribe(nodeHandle_, parameters_.robotTwistTopic_, 1);
  robotTwistCache_.connectInput(robotTwistSubscriber_);
  robotTwistCache_.setCacheSize(parameters_.robotTwistCacheSize_);
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
  nodeHandle.param("robot_twist_topic", robotTwistTopic_, string("/starleth/robot_state/twist"));
  nodeHandle.param("robot_twist_cache_size", robotTwistCacheSize_, 100);
  nodeHandle.param("sensor_cutoff_min_depth", sensorCutoffMaxDepth_, 0.2);
  nodeHandle.param("sensor_cutoff_max_depth", sensorCutoffMaxDepth_, 2.0);
  nodeHandle.param("sensor_model_factor_a", sensorModelFactorA_, 0.003);
  nodeHandle.param("sensor_model_factor_b", sensorModelFactorB_, 0.015);
  nodeHandle.param("sensor_model_factor_c", sensorModelFactorC_, 0.25);
  nodeHandle.param("length_in_x", length_(0), 1.5);
  nodeHandle.param("length_in_y", length_(1), 1.5);
  nodeHandle.param("resolution", resolution_, 0.01);
  nodeHandle.param("min_variance", minVariance_, pow(0.003, 2));
  nodeHandle.param("max_variance", maxVariance_, pow(0.04, 2));
  nodeHandle.param("mahalanobis_distance_threshold", mahalanobisDistanceThreshold_, 2.0);
  nodeHandle.param("multi_height_noise", multiHeightNoise_, pow(0.003, 2));
  nodeHandle.param("bigger_height_threshold_factor", biggerHeightThresholdFactor_, 4.0);
  nodeHandle.param("bigger_height_noise_factor", biggerHeightNoiseFactor_, 2.0);
  nodeHandle.param("robot_twist_variance_factor", robotTwistVarianceFactor_, 1.0);

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
  ROS_ASSERT(timeDependentNoise_ >= 0.0);
  ROS_ASSERT(multiHeightNoise_ >= 0.0);
  ROS_ASSERT(!maxNoUpdateDuration_.isZero());
  ROS_ASSERT(robotTwistCacheSize_ >= 0);
  return true;
}

bool ElevationMapping::initialize()
{
  elevationMapToParentTransform_.setIdentity();
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
  ROS_DEBUG("Updating map with latest prediction from time %f.", robotTwistCache_.getLatestTime().toSec());

  if (time < timeOfLastUpdate_)
  {
    ROS_ERROR("Requested update with time stamp %f, but time of last update was %f.", time.toSec(), timeOfLastUpdate_.toSec());
    return false;
  }
  double timeIntervall = (time - timeOfLastUpdate_).toSec();

  // Variance from motion prediction
  boost::shared_ptr<geometry_msgs::TwistWithCovarianceStamped const> twistMessage = robotTwistCache_.getElemBeforeTime(time);
  if (!twistMessage)
  {
    ROS_ERROR("Could not get twist information from robot for time %f. Buffer empty?", time.toSec());
    return false;
  }

<<<<<<< HEAD
  Matrix<double, 6, 1> twistVariance = Map<const MatrixXd>(twistMessage->twist.covariance.data(), 6, 6).diagonal();

  float variancePrediction = static_cast<float>(timeIntervall * twistVariance.head(3).norm());
=======
  float motionVariance = static_cast<float>(abs(twistVariance->twist.linear.x) * parameters_.robotTwistVarianceFactor_);
>>>>>>> refs/remotes/origin/master

  cout << twistVariance.transpose() << endl;
  cout << timeIntervall << " --> " << variancePrediction << endl << endl;

  // Update map
  if (variancePrediction != 0.0) varianceData_ = (varianceData_.array() + variancePrediction).matrix();

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
  mapUpdateTimer_.setPeriod(parameters_.maxNoUpdateDuration_ - (Time::now() - timeOfLastUpdate_));
  mapUpdateTimer_.start();
}

void ElevationMapping::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace starleth_elevation_map */
