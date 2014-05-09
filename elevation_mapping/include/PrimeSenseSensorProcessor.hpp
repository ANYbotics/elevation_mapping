/*
 * PrimeSenseSensorProcessor.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Core>

// kindr
#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

namespace starleth_elevation_mapping {

/*
 * Sensor processor for PrimeSense structured light sensors.
 *
 * Cleans the point cloud, transforms it to a desired frame, and
 * computes the measurement variances based on a sensor model in
 * the desired frame.
 *
 * Note: Make sure to set the public variables correctly before
 * processing point clouds.
 */
class PrimeSenseSensorProcessor
{
 public:
  PrimeSenseSensorProcessor(tf::TransformListener& transformListener);
  virtual ~PrimeSenseSensorProcessor();

  /*!
   * Processes the point cloud.
   * @param[in] pointCloudInput the input point cloud.
   * @param[in] targetFrame the frame to which the point cloud should be transformed.
   * @param[out] pointCloudOutput the processed point cloud.
   * @param[out] variances the measurement variances expressed in the target frame.
   * @return true if successful.
   */
  bool process(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudInput,
      const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput,
      Eigen::VectorXf& variances);

  //! TF frame id of the map.
  std::string mapFrameId_;

  //! TF frame id of the base.
  std::string baseFrameId_;

  //! Points below the minimal and above the maximal sensor cutoff value are dropped.
  double sensorCutoffMinDepth_;
  double sensorCutoffMaxDepth_;

  /*! PrimeSense sensor model:
   * standardDeviationInNormalDirection = sensorModelNormalFactorA_ + sensorModelNormalFactorB_ * (measurementDistance - sensorModelNormalFactorC_)^2;
   * standardDeviationInLateralDirection = sensorModelLateralFactor_ * measurementDistance
   * Taken from: Nguyen, C. V., Izadi, S., & Lovell, D., Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking, 2012.
   */
  double sensorModelNormalFactorA_, sensorModelNormalFactorB_, sensorModelNormalFactorC_;
  double sensorModelLateralFactor_;

  //! The timeout duration for the lookup of the transformation between sensor frame and target frame.
  ros::Duration transformListenerTimeout_;

  //! Standard horizontal cell variance due to the discretization of the map.
  double discretizationVariance_;

 private:
  bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  bool updateTransformations(std::string sensorFrameId, ros::Time timeStamp);

  bool transformPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTransformed,
      const std::string& targetFrame);

  bool computeVariances(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
      const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
      Eigen::VectorXf& variances);

  tf::TransformListener& transformListener_;

  //! Rotation from Base to Sensor frame (C_SB)
  kindr::rotations::eigen_impl::RotationMatrixPD rotationBaseToSensor_;

  //! Translation from Base to Sensor in Base frame (B_r_BS)
  kindr::phys_quant::eigen_impl::Position3D translationBaseToSensorInBaseFrame_;

  //! Rotation from (elevation) Map to Base frame (C_BM)
  kindr::rotations::eigen_impl::RotationMatrixPD rotationMapToBase_;

  //! Translation from Map to Base in Map frame (M_r_MB)
  kindr::phys_quant::eigen_impl::Position3D translationMapToBaseInMapFrame_;

  //! Transformation from Sensor to Map frame
  Eigen::Affine3d transformationSensorToMap_;

};

} /* namespace starleth_elevation_mapping */
