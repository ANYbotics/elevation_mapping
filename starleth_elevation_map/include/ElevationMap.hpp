/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// StarlETH Navigation
#include <EigenConversions.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

// STD
#include <limits>

namespace starleth_elevation_map {

/*
 * Elevation map stored as planar grid holding elevation height and variance.
 */
class ElevationMap
{
 public:
  ElevationMap(ros::NodeHandle& nodeHandle);

  virtual ~ElevationMap();

  void pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud);

  void mapUpdateTimerCallback(const ros::TimerEvent& timerEvent);

 private:
  bool readParameters();

  bool initialize();

  bool broadcastElevationMapTransform(const ros::Time& time);

  /*!
   * Update the process noise of the elevation map up to a certain time.
   * @param time to which the map is updated to.
   * @return true if successful.
   */
  bool updateProcessNoise(const ros::Time& time);

  double replaceWithNanAtMaxVariance(double x);

  bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
                           const std::string& targetFrame);

  bool addToElevationMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  bool publishElevationMap();

  bool resize(const Eigen::Array2d& length);

  bool reset();

  void setTimeOfLastUpdate(const ros::Time& timeOfLastUpdate);

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber pointCloudSubscriber_;
  ros::Publisher elevationMapPublisher_;
  tf::TransformBroadcaster transformBroadcaster_;
  tf::TransformListener transformListener_;
  ros::Timer mapUpdateTimer_;

  ros::Time timeOfLastUpdate_;

  //! Elevation height data.
  Eigen::MatrixXd elevationData_;

  //! Variance data of the cells in elevationData_.
  Eigen::MatrixXd varianceData_;
  Eigen::MatrixXd varianceDataX_;
  Eigen::MatrixXd varianceDataY_;

  //! Color data.
  Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic> colorData_;

  //! Map size in x, and y-direction [m].
  Eigen::Array2d length_;

  //! Map resolution in xy plane [m/cell].
  double resolution_;

  double minVariance_;
  double maxVariance_;

  ros::Duration maxNoUpdateDuration_;

  //! Origin of the map.
  Eigen::Affine3d elevationMapToParentTransform_;
  std::string parentFrameId_;
  std::string elevationMapFrameId_;

  std::string pointCloudTopic_;

  double sensorCutoffDepth_;

};

template<typename Scalar>
struct VarianceClampOperator
{
  VarianceClampOperator(const Scalar& minVariance, const Scalar& maxVariance)
      : minVariance_(minVariance),
        maxVariance_(maxVariance)
  {
  }
  const Scalar operator()(const Scalar& x) const
  {
    return x < minVariance_ ? minVariance_ : (x > maxVariance_ ? std::numeric_limits<double>::infinity() : x);
  }
  Scalar minVariance_, maxVariance_;
};

} /* namespace starleth_elevation_map */
