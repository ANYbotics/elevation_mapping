/*
 * ElevationMap.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Elevation Mapping
#include <elevation_map_commons/ElevationMapBase.hpp>

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

using namespace elevation_map_commons;

namespace elevation_mapping {

/*!
 * Elevation map stored as planar grid holding elevation height and variance.
 */
class ElevationMap : public ElevationMapBase
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
   * Set the geometry of the elevation map. Resets all the data.
   * @param length the side lengths in x, and y-direction of the elevation map [m].
   * @param position the position of the elevation map in the elevation map frame [m].
   * @param resolution the cell size in [m/cell].
   * @return true if successful.
   */
  bool setGeometry(const Eigen::Array2d& length, const kindr::phys_quant::eigen_impl::Position3D& position,
                   const double& resolution);

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
   * Reset all data of the elevation map (data, lengths, resolution etc.)
   * @return true if successful.
   */
  bool reset();

  const Eigen::MatrixXf& getElevationData();

  const Eigen::MatrixXf& getVarianceData();

  const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& getColorData();

  const Eigen::MatrixXf& getRawElevationData();

  const Eigen::MatrixXf& getRawVarianceData();

  const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& getRawColorData();

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
   * Cumulative distribution function.
   * @param x the argument value.
   * @param mean the mean of the distribution.
   * @param standardDeviation the standardDeviation of the distribution.
   * @return the function value.
   */
  float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

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

  //! Time of last map update.
  ros::Time timeOfLastUpdate_;

  //! Time of last map fusion.
  ros::Time timeOfLastFusion_;

  //! Mutex lock for map fusion process.
  boost::recursive_mutex fusedDataMutex_;

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
