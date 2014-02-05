/*
 * PrimeSenseSensorProcessor.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Core>

namespace starleth_elevation_mapping {

/*
 *
 */
class PrimeSenseSensorProcessor
{
 public:
  PrimeSenseSensorProcessor();
  virtual ~PrimeSenseSensorProcessor();

  bool processPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudInput, const std::string& targetFrame, const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput);

 private:

  bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  bool getMeasurementDistances(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances);

  bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
                           const std::string& targetFrame);

  double sensorCutoffMinDepth_;
  double sensorCutoffMaxDepth_;
  double sensorModelFactorA_;
  double sensorModelFactorB_;
  double sensorModelFactorC_;
};

} /* namespace starleth_elevation_mapping */
