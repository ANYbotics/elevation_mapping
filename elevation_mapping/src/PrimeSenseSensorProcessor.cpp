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
#include <kindr/linear_algebra/LinearAlgebra.hpp>

using namespace std;
using namespace pcl;
using namespace Eigen;
using namespace tf;
using namespace ros;
using namespace kindr;

namespace elevation_mapping {

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
    const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput,
    VectorXf& variances)
{
  PointCloud<PointXYZRGB>::Ptr pointCloudClean(new PointCloud<PointXYZRGB>);
  copyPointCloud(*pointCloudInput, *pointCloudClean);
  cleanPointCloud(pointCloudClean);

  if (!updateTransformations(pointCloudClean->header.frame_id, pointCloudClean->header.stamp)) return false;

  if (!transformPointCloud(pointCloudClean, pointCloudOutput, mapFrameId_)) return false;

  if (!computeVariances(pointCloudClean, robotPoseCovariance, variances)) return false;

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
    const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
    Eigen::VectorXf& variances)
{
  variances.resize(pointCloud->size());

  // Projection vector (P).
  const RowVector3f projectionVector = RowVector3f::UnitZ();

  // Sensor Jacobian (J_s).
  const RowVector3f sensorJacobian = projectionVector * (rotationMapToBase_.transposed() * rotationBaseToSensor_.transposed()).toImplementation().cast<float>();

  // Robot rotation covariance matrix (Sigma_q).
  Matrix3f rotationVariance = robotPoseCovariance.bottomRightCorner(3, 3).cast<float>();

  // Preparations for robot rotation Jacobian (J_q) to minimize computation for every point in point cloud.
  const Matrix3f C_BM_transpose = rotationMapToBase_.transposed().toImplementation().cast<float>();
  const RowVector3f P_mul_C_BM_transpose = projectionVector * C_BM_transpose;
  const Matrix3f C_SB_transpose = rotationBaseToSensor_.transposed().toImplementation().cast<float>();
  const Matrix3f B_r_BS_skew = linear_algebra::getSkewMatrixFromVector(Vector3f(translationBaseToSensorInBaseFrame_.toImplementation().cast<float>()));

  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    // For every point in point cloud.

    // Preparation.
    auto& point = pointCloud->points[i];
    Vector3f pointVector(point.x, point.y, point.z); // S_r_SP
    float heightVariance = 0.0; // sigma_p

    // Measurement distance.
    float measurementDistance = pointVector.norm();

    // Compute sensor covariance matrix (Sigma_S) with sensor model.
    float varianceNormal =
        pow(sensorModelNormalFactorA_ + sensorModelNormalFactorB_ *
        pow(measurementDistance - sensorModelNormalFactorB_, 2), 2);
    float varianceLateral = pow(sensorModelLateralFactor_ * measurementDistance, 2);
    Matrix3f sensorVariance = Matrix3f::Zero();
    sensorVariance.diagonal() << varianceLateral, varianceLateral, varianceNormal;

    // Robot rotation Jacobian (J_q).
    const Matrix3f C_SB_transpose_times_S_r_SP_skew = linear_algebra::getSkewMatrixFromVector(Vector3f(C_SB_transpose * pointVector));
    RowVector3f rotationJacobian = P_mul_C_BM_transpose * (C_SB_transpose_times_S_r_SP_skew + B_r_BS_skew);

    // Measurement variance for map (error propagation law).
    heightVariance = rotationJacobian * rotationVariance * rotationJacobian.transpose();
    heightVariance += sensorJacobian * sensorVariance * sensorJacobian.transpose();

    // Copy to list.
    variances(i) = heightVariance;
  }

  return true;
}

} /* namespace */
