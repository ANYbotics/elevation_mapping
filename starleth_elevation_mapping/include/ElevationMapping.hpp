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
#include "ElevationMap.hpp"
#include "PrimeSenseSensorProcessor.hpp"
#include "CovarianceMapUpdater.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ROS
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

// STD
#include <limits>

namespace starleth_elevation_mapping {

/*
 *
 */
class ElevationMapping
{
 public:
  ElevationMapping(ros::NodeHandle& nodeHandle);

  virtual ~ElevationMapping();

  void pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud);

  void mapUpdateTimerCallback(const ros::TimerEvent& timerEvent);

  bool fuseMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

 private:
  bool readParameters();

  bool initialize();

  bool broadcastElevationMapTransform(const ros::Time& time);

  /*!
   * Update the process noise of the elevation map up to a certain time.
   * @param time to which the map is updated to.
   * @return true if successful.
   */
  bool updatePrediction(const ros::Time& time);

  bool publishRawElevationMap();

  bool publishElevationMap();

  bool updateMapLocation();

  bool getSubmap(starleth_elevation_msg::ElevationSubmap::Request& request, starleth_elevation_msg::ElevationSubmap::Response& response);

  void resetMapUpdateTimer();

  void stopMapUpdateTimer();

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber pointCloudSubscriber_;
  ros::Publisher elevationMapRawPublisher_;
  ros::Publisher elevationMapPublisher_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> robotPoseSubscriber_;
  message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> robotPoseCache_;
  tf::TransformBroadcaster transformBroadcaster_;
  tf::TransformListener transformListener_;
  ros::Timer mapUpdateTimer_;
  ros::ServiceServer fusionTriggerService_;
  ros::ServiceServer submapService_;

  ElevationMap map_;
  PrimeSenseSensorProcessor sensorProcessor_;
  CovarianceMapUpdater mapUpdater_;
  ros::Time timeOfLastUpdate_;
  ros::Time timeOfLastFusion_;


  // Parameters

  //! Maximum time that the map will not be updated.
  ros::Duration maxNoUpdateDuration_;

  //! Duration in which interval the map is checked for relocation.
  ros::Duration mapRelocateTimerDuration_;

  std::string parentFrameId_;
  std::string elevationMapFrameId_;
  std::string trackPointFrameId_;
  std::string trackPointId_;
  std::string pointCloudTopic_;
  std::string robotPoseTopic_;

  kindr::phys_quant::eigen_impl::Position3D trackPoint_;

  int robotPoseCacheSize_;

};

} /* namespace starleth_elevation_map */
