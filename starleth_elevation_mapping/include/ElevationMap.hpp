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

// Schweizer-Messer
#include <sm/timing/Timer.hpp>

namespace starleth_elevation_mapping {

/*
 * Elevation map stored as planar grid holding elevation height and variance.
 */
class ElevationMap
{
 public:
  ElevationMap();
  virtual ~ElevationMap();

  bool setSize(const Eigen::Array2d& length, const double& resolution);

  bool add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::Matrix<float, Eigen::Dynamic, 3>& pointCloudVariances);

  bool update(Eigen::MatrixXf varianceUpdate, Eigen::MatrixXf horizontalVarianceUpdateX, Eigen::MatrixXf horizontalVarianceUpdateY);

  bool fuse();

  bool getSubmap(Eigen::MatrixXf& submap, Eigen::Array2i& centerIndex, const Eigen::MatrixXf& map, const Eigen::Vector2d& center, const Eigen::Array2d& size);

  bool getSubmap(Eigen::MatrixXf& submap, const Eigen::MatrixXf& map, const Eigen::Array2i& topLeftindex, const Eigen::Array2i& size);

  bool relocate(const Eigen::Vector3d& position);

  bool reset();

  const double& getResolution();

  const Eigen::Array2d& getLength();

  const Eigen::Affine3d& getMapToParentTransform();

  const Eigen::Array2i& getBufferStartIndex();

  const Eigen::MatrixXf& getElevationData();

  const Eigen::MatrixXf& getVarianceData();

  const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& getColorData();

  const Eigen::MatrixXf& getRawElevationData();

  const Eigen::MatrixXf& getRawVarianceData();

  const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& getRawColorData();

  Eigen::Array2i getBufferSize();

  // Parameters
  double minVariance_;
  double maxVariance_;
  double mahalanobisDistanceThreshold_;
  double multiHeightNoise_;
  double biggerHeightThresholdFactor_;
  double biggerHeightNoiseFactor_;
  double minHorizontalVariance_;
  double maxHorizontalVariance_;

 private:

  bool clean();

  void resetFusedData();

  void resetCols(unsigned int index, unsigned int nCols);

  void resetRows(unsigned int index, unsigned int nRows);

  float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

  //! Elevation height data.
  Eigen::MatrixXf elevationRawData_;

  //! Variance data of the height of the cells in elevationData_.
  Eigen::MatrixXf varianceRawData_;

  //! Variance data of the cells in elevationData_.
  Eigen::MatrixXf horizontalVarianceRawDataX_;
  Eigen::MatrixXf horizontalVarianceRawDataY_;

  //! Color data.
  Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic> colorRawData_;

  //! The fused map.
  Eigen::MatrixXf elevationData_;
  Eigen::MatrixXf varianceData_;
  Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic> colorData_;

  Eigen::Affine3d toParentTransform_;

  //! Circular buffer start indeces.
  Eigen::Array2i bufferStartIndex_;

  //! Map size in x, and y-direction [m].
  Eigen::Array2d length_;

  //! Map resolution in xy plane [m/cell].
  double resolution_;
};

} /* namespace starleth_elevation_mapping */
