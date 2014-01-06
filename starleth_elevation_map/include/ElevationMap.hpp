/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// StarlETH Navigation
#include <starleth_elevation_msg/ElevationSubmap.h>
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
  bool initialize();

  bool broadcastElevationMapTransform(const ros::Time& time);

  /*!
   * Update the process noise of the elevation map up to a certain time.
   * @param time to which the map is updated to.
   * @return true if successful.
   */
  bool updateProcessNoise(const ros::Time& time);

  bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  bool getMeasurementDistances(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances);

  bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
                           const std::string& targetFrame);

  bool addToElevationMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances);

  bool cleanElevationMap();

  bool publishElevationMap();

  bool resizeMap(const Eigen::Array2d& length);

  bool resetMap();

  bool getSubmap(starleth_elevation_msg::ElevationSubmap::Request& request, starleth_elevation_msg::ElevationSubmap::Response& response);

  void setTimeOfLastUpdate(const ros::Time& timeOfLastUpdate);

  void resetMapUpdateTimer();

  void stopMapUpdateTimer();

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber pointCloudSubscriber_;
  ros::Publisher elevationMapPublisher_;
  tf::TransformBroadcaster transformBroadcaster_;
  tf::TransformListener transformListener_;
  ros::Timer mapUpdateTimer_;
  ros::ServiceServer submapService_;

  ros::Time timeOfLastUpdate_;

  //! Elevation height data.
  Eigen::MatrixXf elevationData_;

  //! Variance data of the cells in elevationData_.
  Eigen::MatrixXf varianceData_;

  //! Color data.
  Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic> colorData_;

  Eigen::Affine3d elevationMapToParentTransform_;

  struct ElevationMapParameters {
    //! Map size in x, and y-direction [m].
    Eigen::Array2d length_;

    //! Map resolution in xy plane [m/cell].
    double resolution_;

    double minVariance_;
    double maxVariance_;

    double mahalanobisDistanceThreshold_;

    double timeProcessNoise_;
    double multiHeightProcessNoise_;

    ros::Duration maxNoUpdateDuration_;

    //! Origin of the map.

    std::string parentFrameId_;
    std::string elevationMapFrameId_;
    std::string pointCloudTopic_;

    double sensorCutoffMinDepth_;
    double sensorCutoffMaxDepth_;

    bool read(ros::NodeHandle& nodeHandle);
    bool checkValidity();
  };

  ElevationMapParameters parameters_;

};

} /* namespace starleth_elevation_map */
