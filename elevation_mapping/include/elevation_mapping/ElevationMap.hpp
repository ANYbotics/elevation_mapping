/*
 * ElevationMap.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Kindr
#include <kindr/Core>

// Boost
#include <boost/thread/recursive_mutex.hpp>

// ROS
#include <ros/ros.h>

namespace elevation_mapping {

/*!
 * Elevation map stored as grid map handling elevation height, variance, color etc.
 */
class ElevationMap
{
 public:

  /*!
   * Constructor.
   */
  ElevationMap(ros::NodeHandle nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ElevationMap();

  /*!
   * Set the geometry of the elevation map. Clears all the data.
   * @param length the side lengths in x, and y-direction of the elevation map [m].
   * @param resolution the cell size in [m/cell].
   * @param position the 2d position of the elevation map in the elevation map frame [m].
   * @return true if successful.
   */
  void setGeometry(const grid_map::Length& length, const double& resolution, const grid_map::Position& position);

  /*!
   * Add new measurements to the elevation map.
   * @param pointCloud the point cloud data.
   * @param pointCloudVariances the corresponding variances of the point cloud data.
   * @return true if successful.
   */
  bool add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances);

  /*!
   * Update the elevation map with variance update data.
   * @param varianceUpdate the variance update in vertical direction.
   * @param horizontalVarianceUpdateX the variance update in horizontal x-direction.
   * @param horizontalVarianceUpdateY the variance update in horizontal y-direction.
   * @param horizontalVarianceUpdateXY the correlated variance update in horizontal xy-direction.
   * @param time the time of the update.
   * @return true if successful.
   */
  bool update(const grid_map::Matrix& varianceUpdate,
              const grid_map::Matrix& horizontalVarianceUpdateX,
              const grid_map::Matrix& horizontalVarianceUpdateY,
              const grid_map::Matrix& horizontalVarianceUpdateXY, const ros::Time& time);

  /*!
   * Triggers the fusion of the entire elevation map.
   * @param computeSurfaceNormals if the surface normals should be computed after the fusion step.
   * @return true if successful.
   */
  bool fuseAll(const bool computeSurfaceNormals);

  /*!
   * Fuses the elevation map for a certain rectangular area.
   * @param position the center position of the area to fuse.
   * @param length the sides lengths of the area to fuse.
   * @param computeSurfaceNormals if the surface normals should be computed after the fusion step.
   * @return true if successful.
   */
  bool fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length, const bool computeSurfaceNormals);

  /*!
   * Clears all data of the elevation map (data and time).
   * @return true if successful.
   */
  bool clear();

  /*!
   * Move the grid map w.r.t. to the grid map frame.
   * @param position the new location of the elevation map in the map frame.
   */
  void move(const Eigen::Vector2d& position);

  /*!
   * Publishes the (latest) raw elevation map.
   * @return true if successful.
   */
  bool publishRawElevationMap();

  /*!
   * Publishes the fused elevation map. Takes the latest available fused elevation
   * map, does not trigger the fusion process.
   * @return true if successful.
   */
  bool publishFusedElevationMap();

  /*!
   * Gets a reference to the raw grid map.
   * @return the raw grid map.
   */
  grid_map::GridMap& getRawGridMap();

  /*!
   * Gets a reference to the fused grid map.
   * @return the fused grid map.
   */
  grid_map::GridMap& getFusedGridMap();

  /*!
   * Gets the time of last map update.
   * @return time of the last map update.
   */
  ros::Time getTimeOfLastUpdate();

  /*!
   * Gets the time of last map fusion.
   * @return time of the last map fusion.
   */
  ros::Time getTimeOfLastFusion();

  /*!
   * Get the pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
   * @return pose of the elevation map frame w.r.t. the parent frame of the robot.
   */
  const kindr::HomTransformQuatD& getPose();

  /*!
   * Gets the position of a raw data point (x, y of cell position & height of cell value) in
   * the parent frame of the robot.
   * @param index the index of the requested cell.
   * @param position the position of the data point in the parent frame of the robot.
   * @return true if successful, false if no valid data available.
   */
  bool getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::Position3D& position);

  /*!
   * Gets the fused data mutex.
   * @return reference to the fused data mutex.
   */
  boost::recursive_mutex& getFusedDataMutex();

  /*!
   * Gets the raw data mutex.
   * @return reference to the raw data mutex.
   */
  boost::recursive_mutex& getRawDataMutex();

  /*!
   * Set the frame id.
   * @param frameId the frame id.
   */
  void setFrameId(const std::string& frameId);

  /*!
   * Get the frame id.
   * @return the frameId.
   */
  const std::string& getFrameId();

  /*!
   * If the raw elevation map has subscribers.
   * @return true if number of subscribers bigger then 0.
   */
  bool hasRawMapSubscribers() const;

  /*!
   * If the fused elevation map has subscribers.
   * @return true if number of subscribers bigger then 0.
   */
  bool hasFusedMapSubscribers() const;

  /*!
   * Callback method for the updates of the underlying map.
   * Updates the internal underlying map.
   * @param underlyingMap the underlying map.
   */
  void underlyingMapCallback(const grid_map_msgs::GridMap& underlyingMap);

  friend class ElevationMapping;

 private:

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Fuses a region of the map.
   * @param topLeftIndex the top left index of the region.
   * @param size the size (in number of cells) of the region.
   * @param computeSurfaceNormals if the surface normals should be computed after the fusion step.
   * @return true if successful.
   */
  bool fuse(const grid_map::Index& topLeftIndex, const grid_map::Index& size, const bool computeSurfaceNormals);

  /*!
   * Computes the surface normals of the fused elevation map for a region of the map.
   * @param topLeftIndex the top left index of the region.
   * @param size the size (in number of cells) of the region.
   * @return true if successful.
   */
  bool computeSurfaceNormals(const Eigen::Array2i& topLeftIndex, const Eigen::Array2i& size);

  /*!
   * Cleans the elevation map data to stay within the specified bounds.
   * @return true if successful.
   */
  bool clean();

  /*!
   * Resets the fused map data.
   * @return true if successful.
   */
  void resetFusedData();

  /*!
   * Cumulative distribution function.
   * @param x the argument value.
   * @param mean the mean of the distribution.
   * @param standardDeviation the standardDeviation of the distribution.
   * @return the function value.
   */
  float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

  //! ROS nodehandle.
  ros::NodeHandle nodeHandle_;

  //! Raw elevation map as grid map.
  grid_map::GridMap rawMap_;

  //! Fused elevation map as grid map.
  grid_map::GridMap fusedMap_;

  //! Underlying map, used for ground truth maps, multi-robot mapping etc.
  grid_map::GridMap underlyingMap_;

  //! True if underlying map has been set, false otherwise.
  bool hasUnderlyingMap_;

  //! Pose of the elevation map frame w.r.t. the inertial parent frame of the robot (e.g. world, map etc.).
  kindr::HomTransformQuatD pose_;

  //! ROS publishers.
  ros::Publisher elevationMapRawPublisher_;
  ros::Publisher elevationMapFusedPublisher_;

  //! Mutex lock for fused map.
  boost::recursive_mutex fusedMapMutex_;

  //! Mutex lock for raw map.
  boost::recursive_mutex rawMapMutex_;

  //! Underlying map subscriber.
  ros::Subscriber underlyingMapSubscriber_;

  //! Parameters. Are set through the ElevationMapping class.
  double minVariance_;
  double maxVariance_;
  double mahalanobisDistanceThreshold_;
  double multiHeightNoise_;
  double minHorizontalVariance_;
  double maxHorizontalVariance_;
  double surfaceNormalEstimationRadius_;
  Eigen::Vector3d surfaceNormalPositiveAxis_;
  std::string underlyingMapTopic_;
};

} /* namespace */
