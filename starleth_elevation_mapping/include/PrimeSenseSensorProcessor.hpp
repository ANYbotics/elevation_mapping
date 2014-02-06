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

  // TODO Check with ASL Styleguide 48b.
  constexpr static unsigned int dimensionOfVariances = 3;

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
      const std::string& targetFrame,
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput,
      Eigen::Matrix<float, Eigen::Dynamic, dimensionOfVariances>& variances);

  //! Points below the minimal and above the maximal sensor cutoff value are dropped.
  double sensorCutoffMinDepth_;
  double sensorCutoffMaxDepth_;

  /*! PrimeSense sensor model:
   * standardDeviation = sensorModelFactorA_ + sensorModelFactorB_ * (measurementDistance - sensorModelFactorC_)^2;
   * Taken from: Nguyen, C. V., Izadi, S., & Lovell, D., Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking, 2012.
   */
  double sensorModelFactorA_;
  double sensorModelFactorB_;
  double sensorModelFactorC_;

  //! The timeout duration for the lookup of the transformation between sensor frame and target frame.
  ros::Duration transformListenerTimeout_;

 private:
  tf::TransformListener& transformListener_;

  bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  bool computeMeasurementDistances(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
      Eigen::VectorXf& measurementDistances);

  bool transformPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
      const std::string& targetFrame);

  bool computeVariances(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
      const Eigen::VectorXf& measurementDistances,
      Eigen::Matrix<float, Eigen::Dynamic, dimensionOfVariances>& variances);

};

} /* namespace starleth_elevation_mapping */
