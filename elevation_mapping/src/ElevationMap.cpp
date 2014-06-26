/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "elevation_mapping/ElevationMap.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMapFunctors.hpp"
#include "elevation_map_msg/TransformationMath.hpp"
#include "elevation_map_msg/ElevationMapMsgHelpers.hpp"

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
using namespace sm;
using namespace sm::timing;

namespace elevation_mapping {

ElevationMap::ElevationMap()
{
  minVariance_ = 0.0;
  maxVariance_ = 0.0;
  mahalanobisDistanceThreshold_ = 0.0;
  multiHeightNoise_ = 0.0;
  minHorizontalVariance_ = 0.0;
  maxHorizontalVariance_ = 0.0;
  reset();
}

ElevationMap::~ElevationMap()
{

}

bool ElevationMap::setGeometry(const Eigen::Array2d& length, const kindr::phys_quant::eigen_impl::Position3D& position, const double& resolution)
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawDataMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedDataMutex_);

  ROS_ASSERT(length(0) > 0.0);
  ROS_ASSERT(length(1) > 0.0);
  ROS_ASSERT(resolution > 0.0);

  int nRows = static_cast<int>(round(length(0) / resolution));
  int nCols = static_cast<int>(round(length(1) / resolution));
  elevationRawData_.resize(nRows, nCols);
  varianceRawData_.resize(nRows, nCols);
  horizontalVarianceRawDataX_.resize(nRows, nCols);
  horizontalVarianceRawDataY_.resize(nRows, nCols);
  colorRawData_.resize(nRows, nCols);
  elevationData_.resize(nRows, nCols);
  varianceData_.resize(nRows, nCols);
  colorData_.resize(nRows, nCols);

  reset();

  resolution_ = resolution;
  length_ = (Array2i(nRows, nCols).cast<double>() * resolution_).matrix();
  position_ = position;

  ROS_DEBUG_STREAM("Elevation map matrix resized to " << elevationRawData_.rows() << " rows and "  << elevationRawData_.cols() << " columns.");
  return true;
}

bool ElevationMap::add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances)
{
  #if ROS_VERSION_MINIMUM(1, 10, 0) // Hydro and newer
    timeOfLastUpdate_.fromNSec(1000.0 * pointCloud->header.stamp); // Double check.
  #else
    timeOfLastUpdate_ = pointCloud->header.stamp;
  #endif

  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];

    Array2i index;
    if (!elevation_map_msg::getIndexFromPosition(
        index, Vector2d(point.x, point.y), length_, position_.vector().head(2), resolution_, getBufferSize(), bufferStartIndex_))
      continue; // Skip this point if it does not lie within the elevation map

    auto& elevation = elevationRawData_(index(0), index(1));
    auto& variance = varianceRawData_(index(0), index(1));
    auto& horizontalVarianceX = horizontalVarianceRawDataX_(index(0), index(1));
    auto& horizontalVarianceY = horizontalVarianceRawDataY_(index(0), index(1));
    auto& color = colorRawData_(index(0), index(1));
    float pointVariance = pointCloudVariances(i);

    if (std::isnan(elevation) || std::isinf(variance)) // TODO Replace with standard method.
    {
      // No prior information in elevation map, use measurement.
      elevation = position_.z() + point.z;
      variance = pointVariance;
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      elevation_map_msg::copyColorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    double mahalanobisDistance = sqrt(pow(point.z - elevation, 2) / variance);

    if (mahalanobisDistance < mahalanobisDistanceThreshold_)
    {
      // Fuse measurement with elevation map data.
      elevation = (variance * (position_.z() + point.z) + pointVariance * elevation) / (variance + pointVariance);
      variance =  (pointVariance * variance) / (pointVariance + variance);
      // TODO Add color fusion.
      elevation_map_msg::copyColorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    // Add noise to cells which have ignored lower values,
    // such that outliers and moving objects are removed
    variance += multiHeightNoise_;

    // Horizontal variances are reset.
    horizontalVarianceX = minHorizontalVariance_;
    horizontalVarianceY = minHorizontalVariance_;
  }

  clean();

  return true;
}

bool ElevationMap::update(Eigen::MatrixXf varianceUpdate, Eigen::MatrixXf horizontalVarianceUpdateX,
                          Eigen::MatrixXf horizontalVarianceUpdateY, const ros::Time& time)
{
  boost::recursive_mutex::scoped_lock scopedLock(rawDataMutex_);

  Array2i bufferSize = getBufferSize();

  if (!(
      (Array2i(varianceUpdate.rows(), varianceUpdate.cols()) == bufferSize).all() &&
      (Array2i(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == bufferSize).all() &&
      (Array2i(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == bufferSize).all()
      ))
  {
    ROS_ERROR("The size of the update matrices does not match.");
    return false;
  }

  varianceRawData_ += varianceUpdate;
  horizontalVarianceRawDataX_ += horizontalVarianceUpdateX;
  horizontalVarianceRawDataY_ += horizontalVarianceUpdateY;
  clean();
  timeOfLastUpdate_ = time;

  return true;
}

bool ElevationMap::fuseAll()
{
  ROS_DEBUG("Requested to fuse entire elevation map.");
  boost::recursive_mutex::scoped_lock scopedLock(fusedDataMutex_);
  return fuse(Array2i(0, 0), getBufferSize());
}

bool ElevationMap::fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length)
{
  ROS_DEBUG("Requested to fuse an area of the elevation map with center at (%f, %f) and side lenghts (%f, %f)",
            position[0], position[1], length[0], length[1]);

  Array2i topLeftIndex;
  Array2i submapBufferSize;

  // These parameters are not used in this function.
  Vector2d submapPosition;
  Array2d submapLength;
  Array2i requestedIndexInSubmap;

  boost::recursive_mutex::scoped_lock scopedLock(fusedDataMutex_);

  elevation_map_msg::getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength,
                                          requestedIndexInSubmap, position, length, length_, position_.vector().head(2),
                                          resolution_, getBufferSize(), bufferStartIndex_);

  return fuse(topLeftIndex, submapBufferSize);
}


bool ElevationMap::fuse(const Eigen::Array2i& topLeftIndex, const Eigen::Array2i& size)
{
  ROS_DEBUG("Fusing elevation map...");

  // Nothing to do.
  if (size.any() == 0) return false;

  // Initializations.
  string timerId = "map_fusion_timer";
  Timer timer(timerId, true);

  boost::recursive_mutex::scoped_lock scopedLock(fusedDataMutex_);

  // Copy raw elevation map data for safe multi-threading.
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawDataMutex_);
  MatrixXf elevationRawDataCopy(elevationRawData_);
  MatrixXf varianceRawDataCopy(varianceRawData_);
  MatrixXf horizontalVarianceRawDataXCopy(horizontalVarianceRawDataX_);
  MatrixXf horizontalVarianceRawDataYCopy(horizontalVarianceRawDataY_);
  Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic> colorRawDataCopy(colorRawData_);
  ros::Time timeOfLastUpdateCopy(timeOfLastUpdate_);
  scopedLockForRawData.unlock();

  // Check if there is the need to reset out-dated data.
  if (timeOfLastFusion_ != timeOfLastUpdateCopy)
  {
    resetFusedData();
  }

  Array2i index;

  // For each cell in requested area.
  for (Array2i areaIndex = Array2i::Zero();
      elevation_map_msg::incrementIndexForSubmap(areaIndex, index, topLeftIndex, size, getBufferSize(),
                                                 bufferStartIndex_);)
  {
    if (timer.isTiming()) timer.stop();
    timer.start();

    // Iteration parameters.
    unsigned int i = index[0];
    unsigned int j = index[1];

    // Check if fusion for this cell has already been done earlier.
    if (!std::isnan(elevationData_(i, j))) continue;

    if (std::isnan(elevationRawDataCopy(i, j)) ||
        std::isinf(varianceRawDataCopy(i, j)) ||
        std::isinf(horizontalVarianceRawDataXCopy(i, j)) ||
        std::isinf(horizontalVarianceRawDataYCopy(i, j)))
    {
      // This is an empty cell (hole in the map).
      continue;
    }

    // Size of submap (2 sigma bound). TODO Add minimum submap size?
    Array2d requestedSubmapLength = 4.0 * Array2d(horizontalVarianceRawDataXCopy(i, j), horizontalVarianceRawDataYCopy(i, j)).sqrt();

    // Requested position (center) of submap in map.
    Vector2d requestedSubmapPosition;
    elevation_map_msg::getPositionFromIndex(requestedSubmapPosition, Array2i(i, j), length_, position_.vector().head(2),
                                            resolution_, getBufferSize(), bufferStartIndex_);

    // Get submap data.
    MatrixXf elevationSubmap, variancesSubmap, horizontalVariancesXSubmap, horizontalVariancesYSubmap;
    Vector2d submapPosition;
    Array2d submapLength;
    Array2i submapBufferSize;
    Array2i requestedIndex;

    getSubmap(elevationSubmap, submapPosition, submapLength, submapBufferSize, requestedIndex, elevationRawDataCopy, requestedSubmapPosition, requestedSubmapLength);
    getSubmap(variancesSubmap, submapPosition, submapLength, submapBufferSize, requestedIndex, varianceRawDataCopy, requestedSubmapPosition, requestedSubmapLength);
    getSubmap(horizontalVariancesXSubmap, submapPosition, submapLength, submapBufferSize, requestedIndex, horizontalVarianceRawDataXCopy, requestedSubmapPosition, requestedSubmapLength);
    getSubmap(horizontalVariancesYSubmap, submapPosition, submapLength, submapBufferSize, requestedIndex, horizontalVarianceRawDataYCopy, requestedSubmapPosition, requestedSubmapLength);

    // Prepare data fusion.
    ArrayXf means, variances, weights;
    int maxNumberOfCellsToFuse = submapBufferSize.prod();
    means.resize(maxNumberOfCellsToFuse);
    variances.resize(maxNumberOfCellsToFuse);
    weights.resize(maxNumberOfCellsToFuse);

    // Position of center index in submap.
    Vector2d centerInSubmap;
    elevation_map_msg::getPositionFromIndex(centerInSubmap, requestedIndex, submapLength, Vector2d::Zero(), resolution_, submapBufferSize,
                                                 Array2i(0, 0));

    unsigned int n = 0;

    // For each cell in submap.
    for (unsigned int p = 0; p < elevationSubmap.rows(); ++p)
    {
      for (unsigned int q = 0; q < elevationSubmap.cols(); ++q)
      {
        if (std::isnan(elevationSubmap(p, q)) ||
            std::isinf(variancesSubmap(p, q)) ||
            std::isinf(horizontalVariancesXSubmap(p, q)) ||
            std::isinf(horizontalVariancesYSubmap(p, q)))
        {
          // Empty cell in submap (cannot be center cell because we checked above).
          continue;
        }

        means[n] = elevationSubmap(p, q);
        variances[n] = variancesSubmap(p, q);

        // Compute weight from probability.
        Vector2d positionInSubmap;
        elevation_map_msg::getPositionFromIndex(positionInSubmap, Array2i(p, q),
                                                     submapLength, Vector2d::Zero(), resolution_,
                                                     submapBufferSize, Array2i(0, 0));

        Vector2d distanceToCenter = (positionInSubmap - centerInSubmap).cwiseAbs();

        float probabilityX =
              cumulativeDistributionFunction(distanceToCenter.x() + resolution_ / 2.0, 0.0, sqrt(horizontalVariancesXSubmap(p, q)))
            - cumulativeDistributionFunction(distanceToCenter.x() - resolution_ / 2.0, 0.0, sqrt(horizontalVariancesXSubmap(p, q)));
        float probabilityY =
              cumulativeDistributionFunction(distanceToCenter.y() + resolution_ / 2.0, 0.0, sqrt(horizontalVariancesYSubmap(p, q)))
            - cumulativeDistributionFunction(distanceToCenter.y() - resolution_ / 2.0, 0.0, sqrt(horizontalVariancesYSubmap(p, q)));

        weights[n] = probabilityX * probabilityY;

        n++;
      }
    }

    if (n == 0)
    {
      // Nothing to fuse.
      elevationData_(i, j) = elevationRawDataCopy(i, j);
      varianceData_(i, j) = varianceRawDataCopy(i, j);
      colorData_(i, j) = colorRawDataCopy(i, j);
      continue;
    }

    // Fuse.
    means.conservativeResize(n);
    variances.conservativeResize(n);
    weights.conservativeResize(n);

    float mean = (weights * means).sum() / weights.sum();
    float variance = (weights * (variances.square() + means.square())).sum() / weights.sum() - pow(mean, 2);

    if (!(std::isfinite(variance) && std::isfinite(mean)))
    {
      ROS_ERROR("Something went wrong when fusing the map: Mean = %f, Variance = %f", mean, variance);
      continue;
    }

    // Add to fused map.
    elevationData_(i, j) = mean;
    varianceData_(i, j) = variance;

    // TODO Add fusion of colors.
    colorData_(i, j) = colorRawDataCopy(i, j);

    timer.stop();
  }

  timeOfLastFusion_ = timeOfLastUpdateCopy;

  ROS_INFO("Elevation map has been fused in %f s.", Timing::getTotalSeconds(timerId));
  ROS_DEBUG("Mean: %f s, Min: %f s, Max: %f s.", Timing::getMeanSeconds(timerId), Timing::getMinSeconds(timerId), Timing::getMaxSeconds(timerId));
  Timing::reset(timerId);

  return true;
}

bool ElevationMap::getSubmap(Eigen::MatrixXf& submap, Eigen::Vector2d& submapPosition, Eigen::Array2d& submapLength, Eigen::Array2i& submapBufferSize,
                             Eigen::Array2i& requestedIndexInSubmap, const Eigen::MatrixXf& map,
                             const Eigen::Vector2d& requestedSubmapPosition, const Eigen::Array2d& requestedSubmapLength)
{
  Array2i topLeftIndex;

  elevation_map_msg::getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap,
                                               requestedSubmapPosition, requestedSubmapLength, length_, position_.vector().head(2), resolution_,
                                               getBufferSize(), bufferStartIndex_);

  std::vector<Eigen::Array2i> submapIndeces;
  std::vector<Eigen::Array2i> submapSizes;

  if(!elevation_map_msg::getBufferRegionsForSubmap(submapIndeces, submapSizes, topLeftIndex, submapBufferSize, getBufferSize(), bufferStartIndex_))
  {
     ROS_ERROR("Cannot access submap of this size.");
     return false;
  }

  unsigned int nRows =
      (submapSizes[3](0) + submapSizes[1](0) > submapSizes[2](0) + submapSizes[0](0)) ?
          submapSizes[3](0) + submapSizes[1](0) : submapSizes[2](0) + submapSizes[0](0);
  unsigned int nCols =
      (submapSizes[3](1) + submapSizes[2](1) > submapSizes[1](1) + submapSizes[0](1)) ?
          submapSizes[3](1) + submapSizes[2](1) : submapSizes[1](1) + submapSizes[0](1);

  submap.resize(nRows, nCols);

  submap.topLeftCorner    (submapSizes[0](0), submapSizes[0](1)) = map.block(submapIndeces[0](0), submapIndeces[0](1), submapSizes[0](0), submapSizes[0](1));
  submap.topRightCorner   (submapSizes[1](0), submapSizes[1](1)) = map.block(submapIndeces[1](0), submapIndeces[1](1), submapSizes[1](0), submapSizes[1](1));
  submap.bottomLeftCorner (submapSizes[2](0), submapSizes[2](1)) = map.block(submapIndeces[2](0), submapIndeces[2](1), submapSizes[2](0), submapSizes[2](1));
  submap.bottomRightCorner(submapSizes[3](0), submapSizes[3](1)) = map.block(submapIndeces[3](0), submapIndeces[3](1), submapSizes[3](0), submapSizes[3](1));

  return true;
}

bool ElevationMap::relocate(const kindr::phys_quant::eigen_impl::Position3D& mapFramePosition)
{
  // Do not relocate if data handling is in process.
  boost::recursive_mutex::scoped_try_lock scopedLockForFusedData(fusedDataMutex_);
  boost::recursive_mutex::scoped_try_lock scopedLockForRawData(rawDataMutex_);
  if(!(scopedLockForFusedData.owns_lock() && scopedLockForRawData)) return false;

  // TODO Add height shift.

  Array2i indexShift;
  Vector2d positionShift = mapFramePosition.vector().head(2) - pose_.getPosition().vector().head(2);
  elevation_map_msg::getIndexShiftFromPositionShift(indexShift, positionShift, resolution_);
  Vector2d alignedPositionShift;
  elevation_map_msg::getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution_);

  // Delete fields that fall out of map (and become empty cells).
  for (int i = 0; i < indexShift.size(); i++)
  {
    if (indexShift[i] != 0)
    {
      if (abs(indexShift[i]) >= getBufferSize()(i))
      {
        // Entire map is dropped.
        elevationRawData_.setConstant(NAN);
        elevationData_.setConstant(NAN);
      }
      else
      {
        // Drop cells out of map.
        int sign = (indexShift[i] > 0 ? 1 : -1);
        int startIndex = bufferStartIndex_[i] - (sign < 0 ? 1 : 0);
        int endIndex = startIndex - sign + indexShift[i];
        int nCells = abs(indexShift[i]);

        int index = (sign > 0 ? startIndex : endIndex);
        elevation_map_msg::mapIndexWithinRange(index, getBufferSize()[i]);

        if (index + nCells <= getBufferSize()[i])
        {
          // One region to drop.
          if (i == 0) resetCols(index, nCells);
          if (i == 1) resetRows(index, nCells);
        }
        else
        {
          // Two regions to drop.
          int firstIndex = index;
          int firstNCells = getBufferSize()[i] - firstIndex;
          if (i == 0) resetCols(firstIndex, firstNCells);
          if (i == 1) resetRows(firstIndex, firstNCells);

          int secondIndex = 0;
          int secondNCells = nCells - firstNCells;
          if (i == 0) resetCols(secondIndex, secondNCells);
          if (i == 1) resetRows(secondIndex, secondNCells);
        }
      }
    }
  }

  // Update information.
  bufferStartIndex_ += indexShift;
  elevation_map_msg::mapIndexWithinRange(bufferStartIndex_, getBufferSize());
  pose_.getPosition() += kindr::phys_quant::eigen_impl::Position3D(alignedPositionShift.x(), alignedPositionShift.y(), 0.0);

  if (indexShift.all() != 0)
    ROS_DEBUG("Elevation has been moved to position (%f, %f).", pose_.getPosition().x(), pose_.getPosition().y());

  return true;
}

bool ElevationMap::reset()
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawDataMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedDataMutex_);

  pose_.setIdentity();
  position_.setZero();
  elevationRawData_.setConstant(NAN);
  varianceRawData_.setConstant(numeric_limits<float>::infinity());
  horizontalVarianceRawDataX_.setConstant(numeric_limits<float>::infinity());
  horizontalVarianceRawDataY_.setConstant(numeric_limits<float>::infinity());
  colorRawData_.setConstant(0);
  elevationData_.setConstant(NAN);
  varianceData_.setConstant(numeric_limits<float>::infinity());
  colorData_.setConstant(0);
  bufferStartIndex_.setZero();
  length_.setZero();
  resolution_ = 0.0;
  resetFusedData();
  timeOfLastUpdate_ = ros::Time::now();
  return true;
}

Eigen::Array2i ElevationMap::getBufferSize()
{
  return Array2i(elevationRawData_.rows(), elevationRawData_.cols());
}

const ros::Time& ElevationMap::getTimeOfLastUpdate()
{
  return timeOfLastUpdate_;
}

const ros::Time& ElevationMap::getTimeOfLastFusion()
{
  boost::recursive_mutex::scoped_lock scopedLock(fusedDataMutex_);
  return timeOfLastFusion_;
}

bool ElevationMap::getDataPointPositionInParentFrame(const Eigen::Array2i& index, kindr::phys_quant::eigen_impl::Position3D& positionInParentFrame)
{
  double height = elevationRawData_(index(0), index(1));
  if(std::isnan(height)) return false;

  Vector2d positionInGrid;
  elevation_map_msg::getPositionFromIndex(positionInGrid, index, length_, position_.vector().head(2), resolution_, getBufferSize(), bufferStartIndex_);

  positionInParentFrame << positionInGrid.x(),
                           positionInGrid.y(),
                           height;

  positionInParentFrame = pose_.transform(positionInParentFrame);

  return true;
}

boost::recursive_mutex& ElevationMap::getFusedDataMutex()
{
  return fusedDataMutex_;
}

boost::recursive_mutex& ElevationMap::getRawDataMutex()
{
  return rawDataMutex_;
}

bool ElevationMap::clean()
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawDataMutex_);
  varianceRawData_ = varianceRawData_.unaryExpr(VarianceClampOperator<double>(minVariance_, maxVariance_));
  horizontalVarianceRawDataX_ = horizontalVarianceRawDataX_.unaryExpr(VarianceClampOperator<double>(minHorizontalVariance_, maxHorizontalVariance_));
  horizontalVarianceRawDataY_ = horizontalVarianceRawDataY_.unaryExpr(VarianceClampOperator<double>(minHorizontalVariance_, maxHorizontalVariance_));
  return true;
}

void ElevationMap::resetFusedData()
{
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedDataMutex_);
  elevationData_.setConstant(NAN);
  varianceData_.setConstant(numeric_limits<float>::infinity());
  colorData_.setConstant(0);
  timeOfLastFusion_.fromSec(0.0);
}

void ElevationMap::resetCols(unsigned int index, unsigned int nCols)
{
  elevationRawData_.block(index, 0, nCols, getBufferSize()[1]).setConstant(NAN);
  elevationData_.block(index, 0, nCols, getBufferSize()[1]).setConstant(NAN);
}

void ElevationMap::resetRows(unsigned int index, unsigned int nRows)
{
  elevationRawData_.block(0, index, getBufferSize()[0], nRows).setConstant(NAN);
  elevationData_.block(0, index, getBufferSize()[0], nRows).setConstant(NAN);
}

const Eigen::Array2d& ElevationMap::getLength()
{
  return length_;
}

const kindr::phys_quant::eigen_impl::Position3D& ElevationMap::getPosition()
{
  return position_;
}

double ElevationMap::getResolution()
{
  return resolution_;
}

const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& ElevationMap::getPose()
{
  return pose_;
}

const Eigen::Array2i& ElevationMap::getBufferStartIndex()
{
  return bufferStartIndex_;
}

const Eigen::MatrixXf& ElevationMap::getElevationData()
{
  boost::recursive_mutex::scoped_lock scopedLock(fusedDataMutex_);
  return elevationData_;
}

const Eigen::MatrixXf& ElevationMap::getVarianceData()
{
  boost::recursive_mutex::scoped_lock scopedLock(fusedDataMutex_);
  return varianceData_;
}

const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& ElevationMap::getColorData()
{
  boost::recursive_mutex::scoped_lock scopedLock(fusedDataMutex_);
  return colorData_;
}

const Eigen::MatrixXf& ElevationMap::getRawElevationData()
{
  return elevationRawData_;
}

const Eigen::MatrixXf& ElevationMap::getRawVarianceData()
{
  return varianceRawData_;
}

const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& ElevationMap::getRawColorData()
{
  return colorRawData_;
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation)
{
  return 0.5 * erfc(-(x-mean)/(standardDeviation*sqrt(2.0)));
}

} /* namespace */
