/*
 * PrimeSenseSensorProcessor.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "PrimeSenseSensorProcessor.hpp"

// PCL
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>

using namespace pcl;
using namespace Eigen;

namespace starleth_elevation_mapping {

PrimeSenseSensorProcessor::PrimeSenseSensorProcessor()
{
  // TODO Auto-generated constructor stub

}

PrimeSenseSensorProcessor::~PrimeSenseSensorProcessor()
{
  // TODO Auto-generated destructor stub
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

  // TODO add sm
//  ROS_DEBUG("ElevationMap: cleanPointCloud() reduced point cloud to %i points.", static_cast<int>(pointCloud->size()));
  return true;
}

bool PrimeSenseSensorProcessor::getMeasurementDistances(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances)
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

bool PrimeSenseSensorProcessor::transformPointCloud(
    const PointCloud<PointXYZRGB>::Ptr pointCloud,
    const std::string& targetFrame)
{
//  StampedTransform transformTf;
//  string sourceFrame = pointCloud->header.frame_id;
//  Time timeStamp =  pointCloud->header.stamp;
//
//  PointCloud<PointXYZRGB>::Ptr pointCloudTransformed(new PointCloud<PointXYZRGB>);
//
//  try
//  {
//    transformListener_.waitForTransform(targetFrame, sourceFrame, timeStamp, ros::Duration(parameters_.maxNoUpdateDuration_));
//    transformListener_.lookupTransform(targetFrame, sourceFrame, timeStamp, transformTf);
//    Affine3d transform;
//    poseTFToEigen(transformTf, transform);
//    pcl::transformPointCloud(*pointCloud, *pointCloudTransformed, transform.cast<float>());
//    pointCloud->swap(*pointCloudTransformed);
//    pointCloud->header.frame_id = targetFrame;
////    pointCloud->header.stamp = timeStamp;
//    ROS_DEBUG("ElevationMap: Point cloud transformed for time stamp %f.", timeStamp.toSec());
//    return true;
//  }
//  catch (TransformException &ex)
//  {
//    ROS_ERROR("%s", ex.what());
//    return false;
//  }
}


} /* namespace starleth_elevation_mapping */
