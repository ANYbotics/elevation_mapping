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

// Kindr
#include <kindr/quaternions/QuaternionEigen.hpp>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace tf;
using namespace ros;

namespace starleth_elevation_mapping {

PrimeSenseSensorProcessor::PrimeSenseSensorProcessor(tf::TransformListener& transformListener)
    : transformListener_(transformListener)
{
  mapFrameId_ = "";
  baseFrameId_ = "";
  transformationSensorToMap_.setIdentity();
  sensorCutoffMinDepth_ = 0.0;
  sensorCutoffMaxDepth_ = 0.0;
  sensorModelNormalFactorA_ = 0.0;
  sensorModelNormalFactorB_ = 0.0;
  sensorModelNormalFactorC_ = 0.0;
  sensorModelLateralFactor_ = 0.0;
  discretizationVariance_ = 0.0;
  transformListenerTimeout_.fromSec(1.0);
}

PrimeSenseSensorProcessor::~PrimeSenseSensorProcessor()
{

}

bool PrimeSenseSensorProcessor::process(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudInput,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput,
    Eigen::Matrix<float, Eigen::Dynamic, dimensionOfVariances>& variances)
{
  PointCloud<PointXYZRGB>::Ptr pointCloudClean(new PointCloud<PointXYZRGB>);
  copyPointCloud(*pointCloudInput, *pointCloudClean);
  cleanPointCloud(pointCloudClean);

  if (!updateTransformations(pointCloudClean->header.frame_id, pointCloudClean->header.stamp)) return false;

  if (!transformPointCloud(pointCloudClean, pointCloudOutput, mapFrameId_)) return false;

  computeVariances(pointCloudClean, variances);

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

bool PrimeSenseSensorProcessor::updateTransformations(std::string sensorFrameId, ros::Time timeStamp)
{
  try
  {
    transformListener_.waitForTransform(sensorFrameId, mapFrameId_, timeStamp, Duration(1.0));

    StampedTransform transformTf;
    transformListener_.lookupTransform(mapFrameId_, sensorFrameId, timeStamp, transformTf);
    poseTFToEigen(transformTf, transformationSensorToMap_);

    transformListener_.lookupTransform(baseFrameId_, sensorFrameId, timeStamp, transformTf); // TODO Why wrong direction?
    Affine3d transform;
    poseTFToEigen(transformTf, transform);
    rotationBaseToSensor_.setMatrix(transform.rotation().matrix());
    translationBaseToSensorInBaseFrame_.toImplementation() = transform.translation();

    transformListener_.lookupTransform(mapFrameId_, baseFrameId_, timeStamp, transformTf); // TODO Why wrong direction?
    poseTFToEigen(transformTf, transform);
    rotationMapToBase_.setMatrix(transform.rotation().matrix());
    translationMapToBaseInMapFrame_.toImplementation() = transform.translation();

    return true;
  }
  catch (TransformException &ex)
  {
    ROS_ERROR("%s", ex.what());
    return false;
  }
}

bool PrimeSenseSensorProcessor::transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTransformed,
    const std::string& targetFrame)
{
  pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transformationSensorToMap_.cast<float>());
  pointCloudTransformed->header.frame_id = targetFrame;
  ROS_DEBUG("ElevationMap: Point cloud transformed to frame %s for time stamp %f.", targetFrame.c_str(),
            pointCloudTransformed->header.stamp.toSec());
  return true;
}

bool PrimeSenseSensorProcessor::computeVariances(
    const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
    Eigen::Matrix<float, Eigen::Dynamic, dimensionOfVariances>& variances)
{
  variances.resize(pointCloud->size(), dimensionOfVariances);

  // Projection vector (P)
  const RowVector3f projectionMatrix = RowVector3f::UnitZ();

  // Sensor Jacobian (J_S)
  Matrix3f jacobianSensor = (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

  // Discretization variance
  Matrix3f discretizationVarianceMatrix = Matrix3f::Zero();
  discretizationVarianceMatrix(0, 0) = discretizationVariance_;
  discretizationVarianceMatrix(1, 1) = discretizationVariance_;

  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    // For every point in point cloud

    // Preparation
    auto& point = pointCloud->points[i];
    Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
    Matrix3f varianceMatrix = Matrix3f::Zero();

    // Measurement distance
    float measurementDistance = pointVector.norm();

    // Compute sensor covariance matrix (Sigma_S) with sensor model
    float varianceNormal =
        pow(sensorModelNormalFactorA_ + sensorModelNormalFactorB_ *
        pow(measurementDistance - sensorModelNormalFactorB_, 2), 2);
    float varianceLateral = pow(sensorModelLateralFactor_ * measurementDistance, 2);
    Matrix3f sensorVariance = Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;



    varianceMatrix = jacobianSensor * sensorVariance * jacobianSensor.transpose() + discretizationVarianceMatrix;

    // Copy to list
    variances.row(i) = varianceMatrix.diagonal();
  }

  return true;
}

} /* namespace starleth_elevation_mapping */
