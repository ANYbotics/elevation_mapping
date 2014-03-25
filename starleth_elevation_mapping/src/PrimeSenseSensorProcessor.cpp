/*
 * PrimeSenseSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "PrimeSenseSensorProcessor.hpp"

// STD
#include <string>

// PCL
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

// ROS
#include <tf_conversions/tf_eigen.h>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace tf;
using namespace ros;

namespace starleth_elevation_mapping {

PrimeSenseSensorProcessor::PrimeSenseSensorProcessor(tf::TransformListener& transformListener)
    : transformListener_(transformListener)
{
  sensorCutoffMinDepth_ = 0.0;
  sensorCutoffMaxDepth_ = 0.0;
  sensorModelNormalFactorA_ = 0.0;
  sensorModelNormalFactorB_ = 0.0;
  sensorModelNormalFactorC_ = 0.0;
  sensorModelLateralFactor_ = 0.0;
  transformListenerTimeout_.fromSec(1.0);
}

PrimeSenseSensorProcessor::~PrimeSenseSensorProcessor()
{

}

bool PrimeSenseSensorProcessor::process(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudInput,
    const std::string& targetFrame,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput,
    Eigen::Matrix<float, Eigen::Dynamic, dimensionOfVariances>& variances)
{
  copyPointCloud(*pointCloudInput, *pointCloudOutput);
  cleanPointCloud(pointCloudOutput);

  VectorXf measurementDistances;
  computeMeasurementDistances(pointCloudOutput, measurementDistances);

  if (!transformPointCloud(pointCloudOutput, targetFrame)) return false;

  computeVariances(pointCloudOutput, measurementDistances, variances);

  return true;
}


bool PrimeSenseSensorProcessor::cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud)
{
  PassThrough<PointXYZRGB> passThroughFilter;
  PointCloud<PointXYZRGB> tempPointCloud;

  passThroughFilter.setInputCloud(pointCloud);
  passThroughFilter.setFilterFieldName("z");
  passThroughFilter.setFilterLimits(sensorCutoffMinDepth_, sensorCutoffMaxDepth_);
  // This makes the point cloud also dense (no NaN points).
  passThroughFilter.filter(tempPointCloud);
  tempPointCloud.is_dense = true;
  pointCloud->swap(tempPointCloud);

  ROS_DEBUG("ElevationMap: cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
  return true;
}

bool PrimeSenseSensorProcessor::computeMeasurementDistances(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
    Eigen::VectorXf& measurementDistances)
{
  // TODO Measurement distances should be added to the point cloud instead of separate data structure.
  measurementDistances.resize(pointCloud->size());

  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];
    measurementDistances[i] = Vector3f(point.x, point.y, point.z).norm();
  }

  return true;
}

bool PrimeSenseSensorProcessor::transformPointCloud(
    const PointCloud<PointXYZRGB>::Ptr pointCloud,
    const std::string& targetFrame)
{
  StampedTransform transformTf;
  string sourceFrame = pointCloud->header.frame_id;
  Time timeStamp =  pointCloud->header.stamp;

  PointCloud<PointXYZRGB>::Ptr pointCloudTransformed(new PointCloud<PointXYZRGB>);

  try
  {
    transformListener_.waitForTransform(targetFrame, sourceFrame, timeStamp, transformListenerTimeout_);
    transformListener_.lookupTransform(targetFrame, sourceFrame, timeStamp, transformTf);
    Affine3d transform;
    poseTFToEigen(transformTf, transform);
    pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
    pointCloud->swap(*pointCloudTransformed);
    pointCloud->header.frame_id = targetFrame;
    pointCloud->header.stamp = timeStamp;
    ROS_DEBUG("ElevationMap: Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(), timeStamp.toSec());
    return true;
  }
  catch (TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool PrimeSenseSensorProcessor::computeVariances(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
    const Eigen::VectorXf& measurementDistances,
    Eigen::Matrix<float, Eigen::Dynamic, dimensionOfVariances>& variances)
{
  variances.resize(measurementDistances.rows(), dimensionOfVariances);

  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];
    auto& measurementDistance = measurementDistances[i];
    RowVector3f variance = RowVector3f::Zero();

    float measurementStandardDeviation = sensorModelNormalFactorA_ + sensorModelNormalFactorB_ * pow(measurementDistance - sensorModelNormalFactorB_, 2);
    variance.z() = pow(measurementStandardDeviation, 2);

    // TODO TODO TODO TODO

    variances.row(i) = variance;
  }

  return true;
}

} /* namespace starleth_elevation_mapping */
