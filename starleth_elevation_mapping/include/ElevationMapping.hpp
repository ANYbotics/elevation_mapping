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
#include <message_filters/cache.h>
#include <message_filters/subscriber.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>

// STD
#include <limits>

namespace starleth_elevation_mapping {

/*
 * Elevation map stored as planar grid holding elevation height and variance.
 */
class ElevationMapping
{
 public:
  ElevationMapping(ros::NodeHandle& nodeHandle);

  virtual ~ElevationMapping();

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
  bool updatePrediction(const ros::Time& time);

  bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  bool getMeasurementDistances(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances);

  bool transformPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
                           const std::string& targetFrame);

  bool addToElevationMap(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& measurementDistances);

  bool cleanElevationMap();

  bool publishElevationMap();

  bool resizeMap(const Eigen::Array2d& length);

  Eigen::Vector2i getMapBufferSize();

  bool resetMap();

  void resetCols(unsigned int index, unsigned int nCols);

  void resetRows(unsigned int index, unsigned int nRows);

  bool updateMapLocation();

  bool relocateMap(const Eigen::Vector3d& position);

  bool getSubmap(starleth_elevation_msg::ElevationSubmap::Request& request, starleth_elevation_msg::ElevationSubmap::Response& response);

  void resetMapUpdateTimer();

  void stopMapUpdateTimer();

  ros::NodeHandle& nodeHandle_;
  ros::Subscriber pointCloudSubscriber_;
  ros::Publisher elevationMapPublisher_;
  message_filters::Subscriber<geometry_msgs::TwistWithCovarianceStamped> robotTwistSubscriber_;
  message_filters::Cache<geometry_msgs::TwistWithCovarianceStamped> robotTwistCache_;
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

  //! Label data.
  Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> labelData_;

  Eigen::Affine3d elevationMapToParentTransform_;

  //! Circular buffer start indeces.
  Eigen::Array2i circularBufferStartIndex_;

  struct ElevationMappingParameters {
    //! Map size in x, and y-direction [m].
    Eigen::Array2d length_;

    //! Map resolution in xy plane [m/cell].
    double resolution_;

    double minVariance_;
    double maxVariance_;

    double mahalanobisDistanceThreshold_;

    double multiHeightNoise_;
    double biggerHeightThresholdFactor_;
    double biggerHeightNoiseFactor_;

    //! Maximum time that the map will not be updated.
    ros::Duration maxNoUpdateDuration_;

    //! Duration in which interval the map is checked for relocation.
    ros::Duration mapRelocateTimerDuration_;

    std::string parentFrameId_;
    std::string elevationMapFrameId_;
    std::string trackPointFrameId_;
    std::string trackPointId_;
    std::string pointCloudTopic_;
    std::string robotTwistTopic_;

    double robotTwistVarianceFactor_;

    Eigen::Vector3d trackPoint_;

    double sensorCutoffMinDepth_;
    double sensorCutoffMaxDepth_;
    double sensorModelFactorA_;
    double sensorModelFactorB_;
    double sensorModelFactorC_;

    int robotTwistCacheSize_;

    bool read(ros::NodeHandle& nodeHandle);
    bool checkValidity();
  };

  ElevationMappingParameters parameters_;

};

} /* namespace starleth_elevation_map */
