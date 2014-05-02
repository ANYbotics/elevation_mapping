/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "ElevationMap.hpp"

// StarlETH Navigation
#include "ElevationMapFunctors.hpp"
#include <TransformationMath.hpp>
#include <ElevationMessageHelpers.hpp>

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

using namespace std;
using namespace Eigen;
using namespace sm;
using namespace sm::timing;

namespace starleth_elevation_mapping {

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

bool ElevationMap::setSize(const Eigen::Array2d& length, const double& resolution)
{
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

  ROS_DEBUG_STREAM("Elevation map matrix resized to " << elevationRawData_.rows() << " rows and "  << elevationRawData_.cols() << " columns.");
  return true;
}

bool ElevationMap::add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances)
{
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];

    Array2i index;
    if (!starleth_elevation_msg::getIndexFromPosition(
        index, Vector2d(point.x, point.y), length_, resolution_, getBufferSize(), bufferStartIndex_))
      continue; // Skip this point if it does not lie within the elevation map

    auto& elevation = elevationRawData_(index(0), index(1));
    auto& variance = varianceRawData_(index(0), index(1));
    auto& horizontalVarianceX = horizontalVarianceRawDataX_(index(0), index(1));
    auto& horizontalVarianceY = horizontalVarianceRawDataY_(index(0), index(1));
    auto& color = colorRawData_(index(0), index(1));
    float pointVariance = pointCloudVariances(i);

    if (std::isnan(elevation) || std::isinf(variance))
    {
      // No prior information in elevation map, use measurement.
      elevation = point.z;
      variance = pointVariance;
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      starleth_elevation_msg::copyColorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    double mahalanobisDistance = sqrt(pow(point.z - elevation, 2) / variance);

    if (mahalanobisDistance < mahalanobisDistanceThreshold_)
    {
      // Fuse measurement with elevation map data.
      elevation = (variance * point.z + pointVariance * elevation) / (variance + pointVariance);
      variance =  (pointVariance * variance) / (pointVariance + variance);
      // TODO add color fusion
      starleth_elevation_msg::copyColorVectorToValue(point.getRGBVector3i(), color);
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

bool ElevationMap::update(Eigen::MatrixXf varianceUpdate, Eigen::MatrixXf horizontalVarianceUpdateX, Eigen::MatrixXf horizontalVarianceUpdateY)
{
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

  return true;
}

bool ElevationMap::fuse()
{
  ROS_DEBUG("Fusing elevation map...");

  // Initializations.
  string timerId = "map_fusion_timer";
  Timer timer(timerId, true);
  resetFusedData();

  // For each cell in map.
  for (unsigned int i = 0; i < elevationData_.rows(); ++i)
  {
    for (unsigned int j = 0; j < elevationData_.cols(); ++j)
    {
      if (timer.isTiming()) timer.stop();
      timer.start();

      if (std::isnan(elevationRawData_(i, j)) ||
          std::isinf(varianceRawData_(i, j)) ||
          std::isinf(horizontalVarianceRawDataX_(i, j)) ||
          std::isinf(horizontalVarianceRawDataY_(i, j)))
      {
        // This is an empty cell (hole in the map).
        continue;
      }

      // Size of submap (2 sigma bound). TODO Add minimum submap size?
      Array2d requestedSubmapLength = 4 * Array2d(horizontalVarianceRawDataX_(i, j), horizontalVarianceRawDataY_(i, j)).sqrt();

      // Requested position (center) of submap in map.
      Vector2d requestedSubmapPosition;
      starleth_elevation_msg::getPositionFromIndex(requestedSubmapPosition, Array2i(i, j),
                                                   length_, resolution_,
                                                   getBufferSize(), bufferStartIndex_);

      // Get submap data.
      MatrixXf elevationSubmap, variancesSubmap, horizontalVariancesXSubmap, horizontalVariancesYSubmap;
      Vector2d submapPosition;
      Array2d submapLength;
      Array2i submapBufferSize;
      Array2i requestedIndex;

      getSubmap(elevationSubmap, submapPosition, submapLength, submapBufferSize, requestedIndex, elevationRawData_, requestedSubmapPosition, requestedSubmapLength);
      getSubmap(variancesSubmap, submapPosition, submapLength, submapBufferSize, requestedIndex, varianceRawData_, requestedSubmapPosition, requestedSubmapLength);
      getSubmap(horizontalVariancesXSubmap, submapPosition, submapLength, submapBufferSize, requestedIndex, horizontalVarianceRawDataX_, requestedSubmapPosition, requestedSubmapLength);
      getSubmap(horizontalVariancesYSubmap, submapPosition, submapLength, submapBufferSize, requestedIndex, horizontalVarianceRawDataY_, requestedSubmapPosition, requestedSubmapLength);

      // Prepare data fusion.
      ArrayXf means, variances, weights;
      int maxNumberOfCellsToFuse = submapBufferSize.prod();
      means.resize(maxNumberOfCellsToFuse);
      variances.resize(maxNumberOfCellsToFuse);
      weights.resize(maxNumberOfCellsToFuse);

      // Position of center index in submap.
      Vector2d centerInSubmap;
      starleth_elevation_msg::getPositionFromIndex(centerInSubmap, requestedIndex, submapLength, resolution_, submapBufferSize,
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
          starleth_elevation_msg::getPositionFromIndex(positionInSubmap, Array2i(p, q),
                                                       submapLength, resolution_,
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
        elevationData_(i, j) = elevationRawData_(i, j);
        varianceData_(i, j) = varianceRawData_(i, j);
        colorData_(i, j) = colorRawData_(i, j);
        continue;
      }

      // Fuse.
      means.conservativeResize(n);
      variances.conservativeResize(n);
      weights.conservativeResize(n);

      float mean = (weights * means).sum() / weights.sum();
      float variance = (weights * (variances.square() + means.square())).sum() / weights.sum() - pow(mean, 2);
      if(variance < 0.0) variance = 0.0; // TODO Why is this necessary?

      if (!(std::isfinite(variance) && std::isfinite(mean)))
      {
        ROS_ERROR("Something went wrong when fusing the map: Mean = %f, Variance = %f", mean, variance);
        continue;
      }

      // Add to fused map.
      elevationData_(i, j) = mean;
      varianceData_(i, j) = variance;

      // TODO Add fusion of colors.
      colorData_(i, j) = colorRawData_(i, j);
      timer.stop();
    }
  }

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

  starleth_elevation_msg::getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap,
                                               requestedSubmapPosition, requestedSubmapLength, length_, resolution_,
                                               getBufferSize(), bufferStartIndex_);

  std::vector<Eigen::Array2i> submapIndeces;
  std::vector<Eigen::Array2i> submapSizes;

  if(!starleth_elevation_msg::getBufferRegionsForSubmap(submapIndeces, submapSizes, topLeftIndex, submapBufferSize, getBufferSize(), bufferStartIndex_))
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

bool ElevationMap::relocate(const kindr::phys_quant::eigen_impl::Position3D& position)
{
  // TODO Add height shift.
  Array2i indexShift;
  Vector2d positionShift = position.vector().head(2) - pose_.getPosition().vector().head(2);
  starleth_elevation_msg::getIndexShiftFromPositionShift(indexShift, positionShift, resolution_);
  Vector2d alignedPositionShift;
  starleth_elevation_msg::getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution_);

  // Delete fields that fall out of map (and become empty cells).
  for (int i = 0; i < indexShift.size(); i++)
  {
    if (indexShift[i] != 0)
    {
      if (abs(indexShift[i]) >= getBufferSize()(i))
      {
        // Entire map is dropped.
        elevationRawData_.setConstant(NAN);
      }
      else
      {
        // Drop cells out of map.
        int sign = (indexShift[i] > 0 ? 1 : -1);
        int startIndex = bufferStartIndex_[i] - (sign < 0 ? 1 : 0);
        int endIndex = startIndex - sign + indexShift[i];
        int nCells = abs(indexShift[i]);

        int index = (sign > 0 ? startIndex : endIndex);
        starleth_elevation_msg::mapIndexWithinRange(index, getBufferSize()[i]);

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
  starleth_elevation_msg::mapIndexWithinRange(bufferStartIndex_, getBufferSize());
  pose_.getPosition() += kindr::phys_quant::eigen_impl::Position3D(alignedPositionShift.x(), alignedPositionShift.y(), 0.0);

  if (indexShift.all() != 0)
    ROS_DEBUG("Elevation has been moved to position (%f, %f).", pose_.getPosition().x(), pose_.getPosition().y());

  return true;
}

bool ElevationMap::reset()
{
  pose_.setIdentity();
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
  return true;
}

Eigen::Array2i ElevationMap::getBufferSize()
{
  return Array2i(elevationRawData_.rows(), elevationRawData_.cols());
}

bool ElevationMap::getPositionInParentFrameFromIndex(const Eigen::Array2i& index, kindr::phys_quant::eigen_impl::Position3D& positionInParentFrame)
{
  double height = elevationRawData_(index(0), index(1));
  if(std::isnan(height)) return false;

  Vector2d positionInGrid;
  starleth_elevation_msg::getPositionFromIndex(positionInGrid, index, length_, resolution_, getBufferSize(), bufferStartIndex_);

  positionInParentFrame << positionInGrid.x(),
                           positionInGrid.y(),
                           height;

  positionInParentFrame = pose_.transform(positionInParentFrame);

  return true;
}

bool ElevationMap::clean()
{
  varianceRawData_ = varianceRawData_.unaryExpr(VarianceClampOperator<double>(minVariance_, maxVariance_));
  horizontalVarianceRawDataX_ = horizontalVarianceRawDataX_.unaryExpr(VarianceClampOperator<double>(minHorizontalVariance_, maxHorizontalVariance_));
  horizontalVarianceRawDataY_ = horizontalVarianceRawDataY_.unaryExpr(VarianceClampOperator<double>(minHorizontalVariance_, maxHorizontalVariance_));
  return true;
}

void ElevationMap::resetFusedData()
{
  elevationData_.setConstant(NAN);
  varianceData_.setConstant(numeric_limits<float>::infinity());
  colorData_.setConstant(0);
}

void ElevationMap::resetCols(unsigned int index, unsigned int nCols)
{
  elevationRawData_.block(index, 0, nCols, getBufferSize()[1]).setConstant(NAN);
}

void ElevationMap::resetRows(unsigned int index, unsigned int nRows)
{
  elevationRawData_.block(0, index, getBufferSize()[0], nRows).setConstant(NAN);
}

const double& ElevationMap::getResolution()
{
  return resolution_;
}

const Eigen::Array2d& ElevationMap::getLength()
{
  return length_;
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
  return elevationData_;
}

const Eigen::MatrixXf& ElevationMap::getVarianceData()
{
  return varianceData_;
}

const Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>& ElevationMap::getColorData()
{
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

} /* namespace starleth_elevation_mapping */
