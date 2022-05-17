/*
 * ElevationMap.hpp
 *
 *  Created on: Nov 12, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

// Grid Map
#include <grid_map_msgs/GetGridMap.h>
#include <grid_map_msgs/ProcessFile.h>
#include <grid_map_msgs/SetGridMap.h>

// ROS
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Boost
#include <boost/thread.hpp>

// Elevation Mapping
#include "elevation_mapping/ElevationMap.hpp"
#include "elevation_mapping/PointXYZRGBConfidenceRatio.hpp"
#include "elevation_mapping/RobotMotionMapUpdater.hpp"
#include "elevation_mapping/ThreadSafeDataWrapper.hpp"
#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"
#include "elevation_mapping/input_sources/InputSourceManager.hpp"
#include "elevation_mapping/sensor_processors/SensorProcessorBase.hpp"

namespace elevation_mapping {

enum class InitializationMethods { PlanarFloorInitializer };

/*!
 * The elevation mapping main class. Coordinates the ROS interfaces, the timing,
 * and the data handling between the other classes.
 */
class ElevationMapping {
 public:
  /*!
   * Constructor.
   *
   * @param nodeHandle the ROS node handle.
   */
  explicit ElevationMapping(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationMapping();

  /*!
   * Callback function for new data to be added to the elevation map.
   *
   * @param pointCloudMsg    The point cloud to be fused with the existing data.
   * @param publishPointCloud If true, publishes the pointcloud after updating the map.
   * @param sensorProcessor_ The sensorProcessor to use in this callback.
   */
  void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& pointCloudMsg, bool publishPointCloud,
                          const SensorProcessorBase::Ptr& sensorProcessor_);

  /*!
   * Callback function for the update timer. Forces an update of the map from
   * the robot's motion if no new measurements are received for a certain time
   * period.
   *
   * @param timerEvent    The timer event.
   */
  void mapUpdateTimerCallback(const ros::TimerEvent& timerEvent);

  /*!
   * Callback function for the fused map publish timer. Publishes the fused map
   * based on configurable duration.
   *
   * @param timerEvent    The timer event.
   */
  void publishFusedMapCallback(const ros::TimerEvent& timerEvent);

  /*!
   * Callback function for cleaning map based on visibility ray tracing.
   *
   * @param timerEvent  The timer event.
   */
  void visibilityCleanupCallback(const ros::TimerEvent& timerEvent);

  /*!
   * ROS service callback function to trigger the fusion of the entire
   * elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool fuseEntireMapServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * ROS service callback function to return a submap of the fused elevation map.
   *
   * @param request     The ROS service request defining the location and size of the fused submap.
   * @param response    The ROS service response containing the requested fused submap.
   * @return true if successful.
   */
  bool getFusedSubmapServiceCallback(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

  /*!
   * ROS service callback function to return a submap of the raw elevation map.
   *
   * @param request     The ROS service request defining the location and size of the raw submap.
   * @param response    The ROS service response containing the requested raw submap.
   * @return true if successful.
   */
  bool getRawSubmapServiceCallback(grid_map_msgs::GetGridMap::Request& request, grid_map_msgs::GetGridMap::Response& response);

  /*!
   * ROS service callback function to enable updates of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool enableUpdatesServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * ROS service callback function to disable updates of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool disableUpdatesServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * ROS service callback function to clear all data of the elevation map.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool clearMapServiceCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

  /*!
   * ROS service callback function to allow for setting the individual layers of the elevation map through a service call.
   * The layer mask can be used to only set certain cells and not the entire map. Cells
   * containing NAN in the mask are not set, all the others are set. If the layer mask is
   * not supplied, the entire map will be set in the intersection of both maps. The
   * provided map can be of different size and position than the map that will be altered.
   *
   * @param request    The ROS service request.
   * @param response   The ROS service response.
   * @return true if successful.
   */
  bool maskedReplaceServiceCallback(grid_map_msgs::SetGridMap::Request& request, grid_map_msgs::SetGridMap::Response& response);

  /*!
   * ROS service callback function to save the grid map with all layers to a ROS bag file.
   *
   * @param request   The ROS service request.
   * @param response  The ROS service response.
   * @return true if successful.
   */
  bool saveMapServiceCallback(grid_map_msgs::ProcessFile::Request& request, grid_map_msgs::ProcessFile::Response& response);

  /*!
   * ROS service callback function to load the grid map with all layers from a ROS bag file.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool loadMapServiceCallback(grid_map_msgs::ProcessFile::Request& request, grid_map_msgs::ProcessFile::Response& response);

  /*!
   * ROS service callback function to reload parameters from the ros parameter server.
   *
   * @param request     The ROS service request.
   * @param response    The ROS service response.
   * @return true if successful.
   */
  bool reloadParametersServiceCallback(std_srvs::Trigger::Request& request, std_srvs::Trigger::Response& response);

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @param reload if enabled disables any geometric resize while reading parameters.
   * Reloading geometry is not supported.
   * @return true if successful.
   */
  bool readParameters(bool reload = false);

  /*!
   * Performs the initialization procedure.
   *
   * @return true if successful.
   */
  bool initialize();

  /**
   * Sets up the subscribers for both robot poses and input data.
   */
  void setupSubscribers();

  /**
   * Sets up the services.
   */
  void setupServices();

  /**
   * Sets up the timers.
   */
  void setupTimers();

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
   *
   * @param time    Time to which the map is updated to.
   * @return true if successful.
   */
  bool updatePrediction(const ros::Time& time);

  /*!
   * Updates the location of the map to follow the tracking point. Takes care
   * of the data handling the goes along with the relocalization.
   *
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

  /*!
   * Initializes a submap around the robot of the elevation map with a constant height.
   */
  bool initializeElevationMap();

  /*!
   * Returns true if fusing the map is enabled.
   */
  bool isFusingEnabled();

  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;

 protected:
  //! Input sources.
  InputSourceManager inputSources_;
  //! ROS subscribers.
  ros::Subscriber pointCloudSubscriber_;  //!< Deprecated, use input_source instead.
  message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> robotPoseSubscriber_;

  //! ROS service servers.
  ros::ServiceServer fusionTriggerService_;
  ros::ServiceServer fusedSubmapService_;
  ros::ServiceServer rawSubmapService_;
  ros::ServiceServer enableUpdatesService_;
  ros::ServiceServer disableUpdatesService_;
  ros::ServiceServer clearMapService_;
  ros::ServiceServer maskedReplaceService_;
  ros::ServiceServer saveMapService_;
  ros::ServiceServer loadMapService_;
  ros::ServiceServer reloadParametersService_;

  //! Callback thread for the fusion services.
  boost::thread fusionServiceThread_;

  //! Callback queue for fusion service thread.
  ros::CallbackQueue fusionServiceQueue_;

  //! Cache for the robot pose messages.
  message_filters::Cache<geometry_msgs::PoseWithCovarianceStamped> robotPoseCache_;

  //! TF listener and broadcaster.
  tf::TransformListener transformListener_;

  struct Parameters {
    //! Size of the cache for the robot pose messages.
    int robotPoseCacheSize_{200};

    //! Frame ID of the elevation map
    std::string mapFrameId_;

    //! Point which the elevation map follows.
    kindr::Position3D trackPoint_;
    std::string trackPointFrameId_;

    //! ROS topics for subscriptions.
    std::string pointCloudTopic_;  //!< Deprecated, use input_source instead.
    std::string robotPoseTopic_;

    //! If true, robot motion updates are ignored.
    bool ignoreRobotMotionUpdates_{false};

    //! If false, elevation mapping stops updating
    bool updatesEnabled_{true};

    //! Maximum time that the map will not be updated.
    ros::Duration maxNoUpdateDuration_;

    //! Time tolerance for updating the map with data before the last update.
    //! This is useful when having multiple sensors adding data to the map.
    ros::Duration timeTolerance_;

    //! Duration for the publishing the fusing map.
    ros::Duration fusedMapPublishTimerDuration_;

    //! If map is fused after every change for debugging/analysis purposes.
    bool isContinuouslyFusing_{false};

    //! Duration for the raytracing cleanup timer.
    ros::Duration visibilityCleanupTimerDuration_;

    //! Name of the mask layer used in the masked replace service
    std::string maskedReplaceServiceMaskLayerName_;

    //! Enables initialization of the elevation map
    bool initializeElevationMap_{false};

    //! Enum to choose the initialization method
    int initializationMethod_{0};

    //! Width of submap of the elevation map with a constant height
    double lengthInXInitSubmap_{1.2};

    //! Height of submap of the elevation map with a constant height
    double lengthInYInitSubmap_{1.8};

    //! Target frame to get the init height of the elevation map
    std::string targetFrameInitSubmap_;

    //! Additional offset of the height value
    double initSubmapHeightOffset_{0.0};

    //! Initial variance when setting a submap.
    double initSubmapVariance_{0.01};
  };
  ThreadSafeDataWrapper<Parameters> parameters_;

  //! Elevation map.
  ElevationMap map_;

  //! Sensor processors. Deprecated use the one from input sources instead.
  SensorProcessorBase::Ptr sensorProcessor_;

  //! Robot motion elevation map updater.
  RobotMotionMapUpdater robotMotionMapUpdater_;

  //! Timer for the robot motion update.
  ros::Timer mapUpdateTimer_;

  //! Time of the last point cloud update.
  ros::Time lastPointCloudUpdateTime_;

  //! Timer for publishing the fused map.
  ros::Timer fusedMapPublishTimer_;

  //! Timer for the raytracing cleanup.
  ros::Timer visibilityCleanupTimer_;

  //! Callback queue for raytracing cleanup thread.
  ros::CallbackQueue visibilityCleanupQueue_;

  //! Callback thread for raytracing cleanup.
  boost::thread visibilityCleanupThread_;

  //! Becomes true when corresponding poses and point clouds can be found
  bool receivedFirstMatchingPointcloudAndPose_;
};

}  // namespace elevation_mapping
