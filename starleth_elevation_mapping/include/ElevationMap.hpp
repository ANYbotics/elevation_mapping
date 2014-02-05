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

namespace starleth_elevation_mapping {

/*
 *
 */
class ElevationMap
{
 public:
  ElevationMap();
  virtual ~ElevationMap();

  bool resize(const Eigen::Array2d& length, const double& resolution);

  bool add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::Matrix<float, Eigen::Dynamic, 3>& variances);

  bool generateFusedMap();

  bool getSubmap(Eigen::MatrixXf& submap, Eigen::Array2i& centerIndex, const Eigen::MatrixXf& map, const Eigen::Vector2d& center, const Eigen::Array2d& size);

  bool getSubmap(Eigen::MatrixXf& submap, const Eigen::MatrixXf& map, const Eigen::Array2i& topLeftindex, const Eigen::Array2i& size);

  bool relocate(const Eigen::Vector3d& position);

  bool reset();


 private:

  Eigen::Vector2i getMapBufferSize();

  bool clean();

  void resetCols(unsigned int index, unsigned int nCols);

  void resetRows(unsigned int index, unsigned int nRows);

  float cumulativeDistributionFunction(float x, float mean, float standardDeviation);

  //! Elevation height data.
  Eigen::MatrixXf elevationData_;

  //! Variance data of the height of the cells in elevationData_.
  Eigen::MatrixXf varianceData_;

  //! Variance data of the cells in elevationData_.
  Eigen::MatrixXf horizontalVarianceDataX_;
  Eigen::MatrixXf horizontalVarianceDataY_;

  //! Color data.
  Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic> colorData_;

  //! Label data.
  Eigen::Matrix<unsigned int, Eigen::Dynamic, Eigen::Dynamic> labelData_;

  Eigen::Affine3d toParentTransform_;

  //! Circular buffer start indeces.
  Eigen::Array2i bufferStartIndex_;

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

};

} /* namespace starleth_elevation_mapping */
