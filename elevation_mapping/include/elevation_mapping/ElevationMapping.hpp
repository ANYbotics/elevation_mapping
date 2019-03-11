/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>

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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_srvs/Empty.h>

// Boost
#include <boost/thread.hpp>


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
   * Callback function for the fused map publish timer. Publishes the fused map
   * based on configurable duration.
   * @param timerEvent the timer event.
   */
  void publishFusedMapCallback(const ros::TimerEvent& timerEvent);


  /*!
   * Callback function for cleaning map based on visibility ray tracing.
   * @param timerEvent the timer event.
   */
  void visibilityCleanupCallback(const ros::TimerEvent& timerEvent);

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
  bool getSubmap(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

  /*!
   * Clears all data of the elevation map.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool clearMap(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * Saves the grid map with all layers to a ROS bag file.
   * @param request the ROS service request.
   * @param response the ROS service response.
   * @return true if successful.
   */
  bool saveMap(grid_map_msgs::ProcessFile::Request& request, grid_map_msgs::ProcessFile::Response& response);

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
   * Separate thread for visibility cleanup.
   */
  void visibilityCleanupThread();

  /*!
   * Update the elevation map from the robot motion up to a certain time.
   * @param time to which the map is updated to.
   * @return true if successful.
   */
  bool updatePrediction(const ros::Time& time);

  /*!
   * Fills a elevation map message with the appropriate header information.
   * @param gridMapMessage the elevation massage to be filled with header information.
   */
  void addHeaderDataToElevationMessage(grid_map_msgs::GridMap& gridMapMessage);

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

  //! ROS service servers.
  ros::ServiceServer fusionTriggerService_;
  ros::ServiceServer submapService_;
  ros::ServiceServer clearMapService_;
  ros::ServiceServer saveMapService_;

  //! Callback thread for the fusion services.
  boost::thread fusionServiceThread_;

  //! Callback queue for fusion service thread.
  ros::CallbackQueue fusionServiceQueue_;

  //! Cache for the robot pose messages.
  message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> robotPoseCache_;

  //! Size of the cache for the robot pose messages.
  int robotPoseCacheSize_;

  //! TF listener and broadcaster.
  tf::TransformListener transformListener_;

  //! Point which the elevation map follows.
  kindr::Position3D trackPoint_;
  std::string trackPointFrameId_;

  //! ROS topics for subscriptions.
  std::string pointCloudTopic_;
  std::string robotPoseTopic_;

  //! Elevation map.
  ElevationMap map_;

  //! Sensor processors.
  SensorProcessorBase::Ptr sensorProcessor_;

  //! Robot motion elevation map updater.
  RobotMotionMapUpdater robotMotionMapUpdater_;

  //! If true, robot motion updates are ignored.
  bool ignoreRobotMotionUpdates_;

  //! Time of the last point cloud update.
  ros::Time lastPointCloudUpdateTime_;

  //! Timer for the robot motion update.
  ros::Timer mapUpdateTimer_;

  //! Maximum time that the map will not be updated.
  ros::Duration maxNoUpdateDuration_;

  //! Time tolerance for updating the map with data before the last update.
  //! This is useful when having multiple sensors adding data to the map.
  ros::Duration timeTolerance_;

  //! Timer for publishing the fused map.
  ros::Timer fusedMapPublishTimer_;

  //! Duration for the publishing the fusing map.
  ros::Duration fusedMapPublishTimerDuration_;

  //! If map is fused after every change for debugging/analysis purposes.
  bool isContinouslyFusing_;

  //! Timer for the raytracing cleanup.
  ros::Timer visibilityCleanupTimer_;

  //! Duration for the raytracing cleanup timer.
  ros::Duration visibilityCleanupTimerDuration_;

  //! Callback queue for raytracing cleanup thread.
  ros::CallbackQueue visibilityCleanupQueue_;

  //! Callback thread for raytracing cleanup.
  boost::thread visibilityCleanupThread_;

  //! Becomes true when corresponding poses and point clouds can be found
  bool receivedFirstMatchingPointcloudAndPose_;
};

} /* namespace */
