/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/PrimeSenseSensorProcessor.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_map_msg/ElevationMap.h"
#include "elevation_map_msg/EigenConversions.hpp"
#include "elevation_map_msg/GetSubmap.h"

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

// Boost
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

// STD
#include <limits>

namespace elevation_mapping {

/*!
 * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
 * and the data handling between the other classes.
 */
class ElevationMapping
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ElevationMapping(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationMapping();

  /*!
   * Callback function for new data to be added to the elevation map.
   * @param pointCloud the point cloud to be fused with the existing data.
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2& pointCloud);

  /*!
   * Callback function for the update timer. Forces an update of the map from
   * the robot's motion if no new measurements are received for a certain time
   * period.
   * @param timerEvent the timer event.
   */
  void mapUpdateTimerCallback(const ros::TimerEvent& timerEvent);

  /*!
   * ROS service callback function to trigger the fusion of the entire
   * elevation map.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool fuseEntireMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * ROS service callback function to return a submap of the elevation map.
   * @param request the ROS service request defining the location and size of the submap.
   * @param response the ROS service response containing the requested submap.
   * @return true if successful.
   */
  bool getSubmap(elevation_map_msg::GetSubmap::Request& request, elevation_map_msg::GetSubmap::Response& response);

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Performs the initialization procedure.
   * @return true if successful.
   */
  bool initialize();

  /*!
   * Separate thread for all fusion service calls.
   */
  void runFusionServiceThread();

  /*!
   * Broadcasts the elevation map to parent transformation.
   * @param time the time of the transformation.
   * @return true if successful.
   */
  bool broadcastElevationMapTransform(const ros::Time& time);

  /*!
   * Update the elevation map from the robot motion up to a certain time.
   * @param time to which the map is updated to.
   * @return true if successful.
   */
  bool updatePrediction(const ros::Time& time);

  /*!
   * Publishes the (latest) raw elevation map.
   * @return true if successful.
   */
  bool publishRawElevationMap();

  /*!
   * Publishes the (fused) elevation map. Takes the latest available (fused) elevation
   * map, does not trigger the fusion process.
   * @return true if successful.
   */
  bool publishElevationMap();

  /*!
   * Fills a elevation map message with the appropriate header information.
   * @param elevationMapMessage the elevation massage to be filled with header information.
   */
  void addHeaderDataToElevationMessage(elevation_map_msg::ElevationMap& elevationMapMessage);

  /*!
   * Updates the location of the map to follow the tracking point. Takes care
   * of the data handling the goes along with the relocalization.
   * @return true if successful.
   */
  bool updateMapLocation();

  /*!
   * Reset and start the map update timer.
   */
  void resetMapUpdateTimer();

  /*!
   * Stop the map update timer.
   */
  void stopMapUpdateTimer();

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! ROS subscribers.
  ros::Subscriber pointCloudSubscriber_;
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> robotPoseSubscriber_;

  //! ROS publishers.
  ros::Publisher elevationMapRawPublisher_;
  ros::Publisher elevationMapPublisher_;

  //! ROS service servers.
  ros::ServiceServer fusionTriggerService_;
  ros::ServiceServer submapService_;

  //! Callback thread for the fusion services.
  boost::thread fusionServiceThread_;

  //! Callback queue for fusion service thread.
  ros::CallbackQueue fusionServiceQueue_;

  //! Cache for the robot pose message.
  message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> robotPoseCache_;
  int robotPoseCacheSize_;

  //! TF listener and broadcaster.
  tf::TransformBroadcaster transformBroadcaster_;
  tf::TransformListener transformListener_;

  //! Frame id of the parent of the elevation map.
  std::string parentFrameId_;

  //! Point which the elevation map follows.
  kindr::phys_quant::eigen_impl::Position3D trackPoint_;
  std::string trackPointFrameId_;

  //! ROS topics for subscriptions.
  std::string pointCloudTopic_;
  std::string robotPoseTopic_;

  //! Elevation map.
  ElevationMap map_;

  //! Sensor processors.
  PrimeSenseSensorProcessor sensorProcessor_;

  //! Robot motion elevation map updater.
  RobotMotionMapUpdater robotMotionMapUpdater_;

  //! Timer for the robot motion update.
  ros::Timer mapUpdateTimer_;

  //! Maximum time that the map will not be updated.
  ros::Duration maxNoUpdateDuration_;

  //! Duration in which interval the map is checked for relocation.
  //! Currently not used, as the map is not relocalized if no new data
  //! is added to the map.
  ros::Duration mapRelocateTimerDuration_;
};

} /* namespace */
