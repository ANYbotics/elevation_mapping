/*
 * ElevationMap.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Kindr
#include <kindr/poses/PoseEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

// Schweizer-Messer
#include <sm/timing/Timer.hpp>

// Boost
#include <boost/thread/recursive_mutex.hpp>

// ROS (time)
#include <ros/ros.h>

namespace elevation_mapping {

/*!
 * Elevation map stored as planar grid holding elevation height and variance.
 */
class ElevationMap
{
 public:

  /*!
   * Constructor.
   */
  ElevationMap();

  /*!
   * Destructor.
   */
  virtual ~ElevationMap();

  /*!
   * Set the geometry of the elevation map. Resets the data.
   * @param length the side lengths in x, and y-direction of the elevation map [m].
   * @param position the position of the elevation map in the map frame [m].
   * @param resolution the cell size in [m/cell].
   * @return true if successful.
   */
  bool setGeometry(const Eigen::Array2d& length, const kindr::phys_quant::eigen_impl::Position3D& position, const double& resolution);

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
   * @param time the time of the update.
   * @return true if successful.
   */
  bool update(Eigen::MatrixXf varianceUpdate, Eigen::MatrixXf horizontalVarianceUpdateX,
              Eigen::MatrixXf horizontalVarianceUpdateY, const ros::Time& time);

  /*!
   * Triggers the fusion of the entire elevation map.
   * @return true if successful.
   */
  bool fuseAll();

  /*!
   * Fuses the elevation map for a certain rectangular area.
   * @param position the center position of the area to fuse.
   * @param length the sides lenghts of the area to fuse.
   * @return true if successful.
   */
  bool fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length);

  /*!
   * Gets a submap from the map. The requested submap is specified with the requested location (usually the center)
   * and length. The returned position of the submap might be different as the submap might not have the same
   * length as the requested length due to the borders of the map.
   * @param[out] submap the data of the submap.
   * @param[out] submapPosition the position of the submap in the map frame.
   * @param[out] submapLength the length of the submap.
   * @param[out] submapBufferSize the buffer size of the submap.
   * @param[out] requestedIndexInSubmap the index of the requested position in the submap.
   * @param[in] map the map to take the submap from.
   * @param[in] requestedSubmapPosition the requested position of the submap.
   * @param[in] requestedSubmapLength the requested length of the submap.
   * @return true if successful.
   */
  bool getSubmap(Eigen::MatrixXf& submap, Eigen::Vector2d& submapPosition, Eigen::Array2d& submapLength, Eigen::Array2i& submapBufferSize,
                 Eigen::Array2i& requestedIndexInSubmap, const Eigen::MatrixXf& map,
                 const Eigen::Vector2d& requestedSubmapPosition, const Eigen::Array2d& requestedSubmapLength);

  /*!
   * Relocate the elevation map frame w.r.t. the parent frame. Use this to move
   * the elevation map boundaries without moving the elevation map data. Takes care
   * of all the data handling, such that the elevation map is stationary in the parent frame.
   * @param mapFramePosition the new location of the elevation map frame in the parent frame.
   * @return true if successful.
   */
  bool relocate(const kindr::phys_quant::eigen_impl::Position3D& mapFramePosition);

  /*!
   * Reset all data of the elevation map (data, lengths, resolution etc.)
   * @return true if successful.
   */
  bool reset();

  /*!
   * Get the side length of the map.
   * @return side length of the map.
   */
  const Eigen::Array2d& getLength();

  /*!
   * Get the position of the map in the elevation map frame.
   * @return position of the map in the elevation map frame.
   */
  const kindr::phys_quant::eigen_impl::Position3D& getPosition();

  /*!
   * Get the resolution of the elevation map.
   * @return resolution of the elevation map in the xy plane [m/cell].
   */
  double getResolution();

  /*!
   * Get the pose of the elevation map frame w.r.t. the parent frame.
   * @return pose of the elevation map frame w.r.t. the parent frame.
   */
  const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& getPose();

  const Eigen::Array2i& getBufferStartIndex();

  const Eigen::MatrixXf& getElevationData();

  const Eigen::MatrixXf& getVarianceData();

  const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& getColorData();

  const Eigen::MatrixXf& getRawElevationData();

  const Eigen::MatrixXf& getRawVarianceData();

  const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& getRawColorData();

  Eigen::Array2i getBufferSize();

  /*!
   * Gets the time of last map update.
   * @return time of the last map update.
   */
  const ros::Time& getTimeOfLastUpdate();

  /*!
   * Gets the time of last map fusion.
   * @return time of the last map fusion.
   */
  const ros::Time& getTimeOfLastFusion();

  /*!
   * Gets the position of a data point (x, y of cell position & height of cell value) in
   * the parent frame of the elevation map.
   * @param index the index of the requested data point.
   * @param positionInParentFrame the position of the data point in the parent frame.
   * @return true if successful, false if no valid data available.
   */
  bool getDataPointPositionInParentFrame(const Eigen::Array2i& index, kindr::phys_quant::eigen_impl::Position3D& positionInParentFrame);

  friend class ElevationMapping;

 private:

  /*!
   * Fuses a region of the map.
   * @param topLeftIndex the top left index of the region.
   * @param size the size (in number of cells) of the region.
   * @return true if successful.
   */
  bool fuse(const Eigen::Array2i& topLeftIndex, const Eigen::Array2i& size);

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
   * Resets a number of columns in the raw elevation map.
   * @param index the left index for the columns to be reset.
   * @param nCols the number of columns to reset.
   */
  void resetCols(unsigned int index, unsigned int nCols);

  /*!
   * Resets a number of rows in the raw elevation map.
   * @param index the upper index for the rows to be reset.
   * @param nRows the number of rows to reset.
   */
  void resetRows(unsigned int index, unsigned int nRows);

  /*!
   * Cumulative distribution function.
   * @param x the argument value.
   * @param mean the mean of the distribution.
   * @param standardDeviation the standardDeviation of the distribution.
   * @return the function value.
   */
  float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

  //! Frame id of the elevation map.
  std::string frameId_;

  //! Pose of the elevation map frame w.r.t. the parent frame.
  kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD pose_;

  //! Map size in x, and y-direction [m].
  Eigen::Array2d length_;

  //! Map position in the elevation map frame [m].
  kindr::phys_quant::eigen_impl::Position3D position_;

  //! Map resolution in xy plane [m/cell].
  double resolution_;

  //! Elevation height raw data.
  Eigen::MatrixXf elevationRawData_;

  //! Variance raw data of the height of the cells in elevationData_.
  Eigen::MatrixXf varianceRawData_;

  //! Variance raw data of the cells in elevationData_.
  Eigen::MatrixXf horizontalVarianceRawDataX_;
  Eigen::MatrixXf horizontalVarianceRawDataY_;

  //! Color raw data.
  Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic> colorRawData_;

  //! The fused map height data.
  Eigen::MatrixXf elevationData_;

  //! The fused map variance data.
  Eigen::MatrixXf varianceData_;

  //! The fused map color data.
  Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic> colorData_;

  //! Circular buffer start indeces.
  Eigen::Array2i bufferStartIndex_;

  //! Time of last map update.
  ros::Time timeOfLastUpdate_;

  //! Time of last map fusion.
  ros::Time timeOfLastFusion_;

  //! Mutex lock for map fusion process.
  boost::recursive_mutex fusionMutex_;

  //! Mutex lock for raw data handling.
  boost::recursive_mutex rawDataMutex_;

  //! Parameters. Are set through the ElevationMapping class.
  double minVariance_;
  double maxVariance_;
  double mahalanobisDistanceThreshold_;
  double multiHeightNoise_;
  double minHorizontalVariance_;
  double maxHorizontalVariance_;
};

} /* namespace */
