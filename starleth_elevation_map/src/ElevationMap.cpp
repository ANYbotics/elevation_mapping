/*
 * ElevationMap.cpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "ElevationMap.hpp"

// StarlETH Navigation
#include "ElevationMappingHelpers.hpp"
#include <ElevationMapHelpers.hpp>
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

namespace starleth_elevation_map {

// TODO Rename this class?

ElevationMap::ElevationMap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  ROS_INFO("StarlETH elevation map node started.");
  parameters_.read(nodeHandle_);
  pointCloudSubscriber_ = nodeHandle_.subscribe(parameters_.pointCloudTopic_, 1, &ElevationMap::pointCloudCallback, this);
  elevationMapPublisher_ = nodeHandle_.advertise<starleth_elevation_msg::ElevationMap>("elevation_map", 1);
  mapUpdateTimer_ = nodeHandle_.createTimer(parameters_.maxNoUpdateDuration_, &ElevationMap::mapUpdateTimerCallback, this, true, false);
  initialize();
}

ElevationMap::~ElevationMap()
{

}

bool ElevationMap::ElevationMapParameters::read(ros::NodeHandle& nodeHandle)
{
  nodeHandle.param("point_cloud_topic", pointCloudTopic_, string("/depth_registered/points_throttled"));
  nodeHandle.param("map_frame_id", parentFrameId_, string("/map"));
  nodeHandle.param("elevation_map_id", elevationMapFrameId_, string("/elevation_map"));
  nodeHandle.param("sensor_cutoff_min_depth", sensorCutoffMaxDepth_, 0.2);
  nodeHandle.param("sensor_cutoff_max_depth", sensorCutoffMaxDepth_, 2.0);
  nodeHandle.param("length_in_x", length_(0), 3.0);
  nodeHandle.param("length_in_y", length_(1), 3.0);
  nodeHandle.param("resolution", resolution_, 0.01);
  nodeHandle.param("min_variance", minVariance_, pow(0.003, 2));
  nodeHandle.param("max_variance", maxVariance_, pow(0.075, 2));
  nodeHandle.param("mahalanobis_distance_threshold", mahalanobisDistanceThreshold_, 2.0);
  nodeHandle.param("time_process_noise", timeProcessNoise_, pow(0.0, 2));
  nodeHandle.param("multi_height_process_noise", multiHeightProcessNoise_, pow(0.01, 2));

  double minUpdateRate;
  nodeHandle.param("min_update_rate", minUpdateRate, 2.0);
  maxNoUpdateDuration_.fromSec(1.0 / minUpdateRate);

  return checkValidity();
}

bool ElevationMap::ElevationMapParameters::checkValidity()
{
  ROS_ASSERT(sensorCutoffMinDepth_ >= 0.0);
  ROS_ASSERT(sensorCutoffMaxDepth_ > sensorCutoffMinDepth_);
  ROS_ASSERT(length_(0) > 0.0);
  ROS_ASSERT(length_(1) > 0.0);
  ROS_ASSERT(resolution_ > 0.0);
  ROS_ASSERT(minVariance_ >= 0.0);
  ROS_ASSERT(maxVariance_ > minVariance_);
  ROS_ASSERT(mahalanobisDistanceThreshold_ >= 0.0);
  ROS_ASSERT(timeProcessNoise_ >= 0.0);
  ROS_ASSERT(multiHeightProcessNoise_ >= 0.0);

  return true;
}

bool ElevationMap::initialize()
{
  // TODO
  elevationMapToParentTransform_.setIdentity();
  elevationMapToParentTransform_.translation().x() = 1.0;

  resizeMap(parameters_.length_);
  resetMap();
  broadcastElevationMapTransform(Time::now());
  Duration(1.0).sleep(); // Need this to get the TF caches fill up.
  setTimeOfLastUpdate(Time::now());
  resetMapUpdateTimer();
  ROS_INFO("StarlETH elevation map node initialized.");
  return true;
}

void ElevationMap::pointCloudCallback(
    const sensor_msgs::PointCloud2& rawPointCloud)
{
  stopMapUpdateTimer();

  // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
  PointCloud<PointXYZRGB>::Ptr pointCloud(new PointCloud<PointXYZRGB>);
  fromROSMsg(rawPointCloud, *pointCloud);
  ROS_DEBUG("ElevationMap received a point cloud (%i points) for elevation mapping.", static_cast<int>(pointCloud->size()));

  Time& time = pointCloud->header.stamp;
  if (!broadcastElevationMapTransform(time)) ROS_ERROR("ElevationMap: Broadcasting elevation map transform to parent failed.");
  if (!updateProcessNoise(time)) { ROS_ERROR("ElevationMap: Updating process noise failed."); return; }
  setTimeOfLastUpdate(time);
  cleanPointCloud(pointCloud);
  VectorXf measurementDistances;
  getMeasurementDistances(pointCloud, measurementDistances);
  if (!transformPointCloud(pointCloud, parameters_.elevationMapFrameId_)) ROS_ERROR("ElevationMap: Point cloud transform failed for time stamp %f.", time.toSec());
  else if (!addToElevationMap(pointCloud, measurementDistances)) ROS_ERROR("ElevationMap: Adding point cloud to elevation map failed.");
  cleanElevationMap();
  if (!publishElevationMap()) ROS_INFO("ElevationMap: Elevation map has not been broadcasted.");
  resetMapUpdateTimer();
}

void ElevationMap::mapUpdateTimerCallback(const ros::TimerEvent& timerEvent)
{
  ROS_WARN("Elevation map is updated without data from the sensor.");

  stopMapUpdateTimer();
  Time time = Time::now();
  if (!broadcastElevationMapTransform(time)) ROS_ERROR("ElevationMap: Broadcasting elevation map transform to parent failed.");
  if (!updateProcessNoise(time)) { ROS_ERROR("ElevationMap: Updating process noise failed."); return; }
  setTimeOfLastUpdate(time);
  cleanElevationMap();
  if (!publishElevationMap()) ROS_ERROR("ElevationMap: Elevation map has not been broadcasted.");
  resetMapUpdateTimer();
}

bool ElevationMap::broadcastElevationMapTransform(const ros::Time& time)
{
  tf::Transform tfTransform;
  poseEigenToTF(elevationMapToParentTransform_, tfTransform);
  transformBroadcaster_.sendTransform(tf::StampedTransform(tfTransform, time, parameters_.parentFrameId_, parameters_.elevationMapFrameId_));
  ROS_DEBUG("Published transform for elevation map in parent frame at time %f.", time.toSec());
  return true;
}

bool ElevationMap::updateProcessNoise(const ros::Time& time)
{
  if (time < timeOfLastUpdate_) return false;

  // TODO Add variance depending on movement from previous update time

  float timeProcessNoise = static_cast<float>((time - timeOfLastUpdate_).toSec() * parameters_.timeProcessNoise_);
  if (timeProcessNoise != 0.0) varianceData_ = (varianceData_.array() + timeProcessNoise).matrix();
  return true;
}

bool ElevationMap::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
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

bool ElevationMap::getMeasurementDistances(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances)
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

bool ElevationMap::transformPointCloud(
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

bool ElevationMap::addToElevationMap(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances)
{
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];

    Array2i index;
    if (!starleth_elevation_msg::getIndexFromPosition(
        index, Vector2d(point.x, point.y), parameters_.length_, parameters_.resolution_))
      continue; // Skip this point if it does not lie within the elevation map

    auto& elevation = elevationData_(index(0), index(1));
    auto& variance = varianceData_(index(0), index(1));
    auto& color = colorData_(index(0), index(1));
    auto& measurementDistance = measurementDistances[i];

    // Kinect (short-range mode) taken from:
    // Nguyen, C. V., Izadi, S., & Lovell, D. (2012).
    // Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking.
//    float measurementStandardDeviation = 0.0012 + 0.0019 * pow(measurementDistance - 0.4, 2);

    // Manual tuned for PrimeSense Carmine 1.09.
    // Also includes uncertainties with non perpedicular surfaces and
    // biases/warp at further distances.
    float measurementStandardDeviation = 0.003 + 0.015 * pow(measurementDistance - 0.25, 2);

    // Approximation of datasheet for Carmine 1.09 from:
    // http://www.openni.org/rd1-09-specifications/
    // Coefficients computed with least squares regression.
//    float measurementStandardDeviation = 0.000181 + 0.00166 * pow(measurementDistance - 0.1, 2);

    float measurementVariance = pow(measurementStandardDeviation, 2);

//    cout << "Sensor: " << measurementStandardDeviation << ", Map: "  << sqrt(variance) << endl;

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

    if (point.z > elevation)
    {
      // Overwrite elevation map information with measurement data
      elevation = point.z;
      variance = measurementVariance;
      starleth_elevation_msg::copyColorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    // TODO Add label to cells which are potentially multi-level

    // Add noise to cells which have ignored lower values,
    // such we outliers and moving objects are removed
    variance += parameters_.multiHeightProcessNoise_;
  }

  return true;
}

bool ElevationMap::cleanElevationMap()
{
  varianceData_ = varianceData_.unaryExpr(VarianceClampOperator<double>(parameters_.minVariance_, parameters_.maxVariance_));
  return true;
}

bool ElevationMap::publishElevationMap()
{
  if (elevationMapPublisher_.getNumSubscribers () < 1) return false;

  starleth_elevation_msg::ElevationMap elevationMapMessage;

  elevationMapMessage.header.stamp = timeOfLastUpdate_;
  elevationMapMessage.header.frame_id = parameters_.elevationMapFrameId_;
  elevationMapMessage.resolution = parameters_.resolution_;
  elevationMapMessage.lengthInX = parameters_.length_(0);
  elevationMapMessage.lengthInY = parameters_.length_(1);

  starleth_elevation_msg::matrixEigenToMultiArrayMessage(elevationData_, elevationMapMessage.elevation);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(varianceData_, elevationMapMessage.variance);
  starleth_elevation_msg::matrixEigenToMultiArrayMessage(colorData_, elevationMapMessage.color);

  elevationMapPublisher_.publish(elevationMapMessage);

  ROS_DEBUG("Elevation map has been published.");

  return true;
}

bool ElevationMap::resizeMap(const Eigen::Array2d& length)
{
  int nRows = static_cast<int>(round(length(0) / parameters_.resolution_));
  int nCols = static_cast<int>(round(length(1) / parameters_.resolution_));
  parameters_.length_ = (Array2i(nRows, nCols).cast<double>() * parameters_.resolution_).matrix();

  elevationData_.resize(nRows, nCols);
  varianceData_.resize(nRows, nCols);
  colorData_.resize(nRows, nCols);

  ROS_DEBUG_STREAM("Elevation map matrix resized to " << elevationData_.rows() << " rows and "  << elevationData_.cols() << " columns.");
  return true;
}

bool ElevationMap::resetMap()
{
  elevationData_.setConstant(NAN);
  varianceData_.setConstant(NAN);
  colorData_.setConstant(0);
  return true;
}

void ElevationMap::setTimeOfLastUpdate(const ros::Time& timeOfLastUpdate)
{
  timeOfLastUpdate_ = timeOfLastUpdate;
}

void ElevationMap::resetMapUpdateTimer()
{
  mapUpdateTimer_.stop();
  mapUpdateTimer_.setPeriod(parameters_.maxNoUpdateDuration_ - (Time::now() - timeOfLastUpdate_));
  mapUpdateTimer_.start();
}

void ElevationMap::stopMapUpdateTimer()
{
  mapUpdateTimer_.stop();
}

} /* namespace starleth_elevation_map */
