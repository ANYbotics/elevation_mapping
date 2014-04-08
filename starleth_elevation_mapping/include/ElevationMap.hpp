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

  bool relocate(const kindr::phys_quant::eigen_impl::Position3D& position);

  bool reset();

  const double& getResolution();

  const Eigen::Array2d& getLength();

  const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& getPose();

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

  kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD pose_;

  //! Circular buffer start indeces.
  Eigen::Array2i bufferStartIndex_;

  //! Map size in x, and y-direction [m].
  Eigen::Array2d length_;

  //! Map resolution in xy plane [m/cell].
  double resolution_;
};

} /* namespace starleth_elevation_mapping */
