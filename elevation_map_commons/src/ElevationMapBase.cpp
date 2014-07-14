/*
 * ElevationMapBase.cpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_map_commons/ElevationMapBase.hpp"

// Elevation Mapping
#include "elevation_map_commons/TransformationMath.hpp"

// ROS
#include "ros/console.h"
#include "ros/assert.h"

using namespace std;
using namespace Eigen;

namespace elevation_map_commons {

ElevationMapBase::ElevationMapBase()
{
  frameId_.clear();
  timeStamp_ = ros::Time(0.0);
  pose_.setIdentity();
  position_.setZero();
  length_.setZero();
  resolution_ = 0.0;
  bufferSize_.setZero();
  bufferStartIndex_.setZero();

  mapFloatData_.insert(std::pair<FloatDataType, MatrixXf>(FloatDataType::Elevation, MatrixXf(0, 0)));

  mapFloatResetValues_.insert(std::pair<FloatDataType, const float>(FloatDataType::Elevation, NAN));
  mapFloatResetValues_.insert(std::pair<FloatDataType, const float>(FloatDataType::Variance, numeric_limits<float>::infinity()));
  mapFloatResetValues_.insert(std::pair<FloatDataType, const float>(FloatDataType::HorizontalVarianceX, numeric_limits<float>::infinity()));
  mapFloatResetValues_.insert(std::pair<FloatDataType, const float>(FloatDataType::HorizontalVarianceY, numeric_limits<float>::infinity()));
  mapIntResetValues_.insert(std::pair<IntDataType, const unsigned long>(IntDataType::Color, 0));
}

ElevationMapBase::~ElevationMapBase()
{

}

bool ElevationMapBase::setGeometry(const Eigen::Array2d& length,
                                   const kindr::phys_quant::eigen_impl::Position3D& position, const double& resolution)
{
  ROS_ASSERT(length(0) > 0.0);
  ROS_ASSERT(length(1) > 0.0);
  ROS_ASSERT(resolution > 0.0);

  Array2i bufferSize;
  bufferSize(0) = static_cast<int>(round(length(0) / resolution)); // There is no round() function in Eigen.
  bufferSize(1) = static_cast<int>(round(length(1) / resolution));
  resizeBuffer(bufferSize);
  resetAll();

  resolution_ = resolution;
  length_ = (bufferSize_.cast<double>() * resolution_).matrix();
  position_ = position;
  bufferStartIndex_.setZero();

  ROS_DEBUG_STREAM("Elevation map matrix resized to " << bufferSize_(0) << " rows and "  << bufferSize_(1) << " columns.");
  return true;
}

void ElevationMapBase::setFrame(
    const std::string& frameId,
    const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& pose)
{
  frameId_ = frameId;
  pose_ = pose;
}

void ElevationMapBase::reset()
{
  mapFloatData_[FloatDataType::Elevation].setConstant(mapFloatResetValues_[FloatDataType::Elevation]);
}

bool ElevationMapBase::relocate(const kindr::phys_quant::eigen_impl::Position3D& mapFramePosition)
{
  // TODO Add height shift.

  Array2i indexShift;
  Vector2d positionShift = mapFramePosition.vector().head(2) - pose_.getPosition().vector().head(2);
  getIndexShiftFromPositionShift(indexShift, positionShift, resolution_);
  Vector2d alignedPositionShift;
  getPositionShiftFromIndexShift(alignedPositionShift, indexShift, resolution_);

  // Delete fields that fall out of map (and become empty cells).
  for (int i = 0; i < indexShift.size(); i++)
  {
    if (indexShift[i] != 0)
    {
      if (abs(indexShift[i]) >= getBufferSize()(i))
      {
        // Entire map is dropped.
        reset();
      }
      else
      {
        // Drop cells out of map.
        int sign = (indexShift[i] > 0 ? 1 : -1);
        int startIndex = bufferStartIndex_[i] - (sign < 0 ? 1 : 0);
        int endIndex = startIndex - sign + indexShift[i];
        int nCells = abs(indexShift[i]);

        int index = (sign > 0 ? startIndex : endIndex);
        mapIndexWithinRange(index, getBufferSize()[i]);

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
  mapIndexWithinRange(bufferStartIndex_, getBufferSize());
  pose_.getPosition() += kindr::phys_quant::eigen_impl::Position3D(alignedPositionShift.x(), alignedPositionShift.y(), 0.0);

  if (indexShift.all() != 0)
    ROS_DEBUG("Elevation has been moved to position (%f, %f).", pose_.getPosition().x(), pose_.getPosition().y());

  return true;
}

ElevationMapBase ElevationMapBase::getSubmap(const Eigen::Vector2d& position, const Eigen::Array2d& length,
                                             Eigen::Array2i& indexInSubmap, bool& success)
{

  // Submap the generate.
  ElevationMapBase submap;

  // Get submap geometric information.
  // TODO Position handling not very nice.
  Vector2d mapPosition = position_.getHead(2).vector();
  Vector2d submapPosition = submap.position_.getHead(2).vector();
  Array2i topLeftIndex;

  getSubmapInformation(topLeftIndex, submap.bufferSize_, submapPosition, submap.length_, indexInSubmap, position,
                       length, length_, mapPosition, resolution_, bufferSize_, bufferStartIndex_);

  submap.position_.x() = submapPosition.x();
  submap.position_.y() = submapPosition.y();
  submap.position_.z() = position_.z();

  // Copy data.
  std::vector<Eigen::Array2i> submapIndeces;
  std::vector<Eigen::Array2i> submapSizes;

  if(!getBufferRegionsForSubmap(submapIndeces, submapSizes, topLeftIndex, submap.bufferSize_, bufferSize_, bufferStartIndex_))
  {
     ROS_ERROR("Cannot access submap of this size.");
     success = false;
     return ElevationMapBase();
  }

  Array2i submapBufferSize;
  submapBufferSize[0] =
      (submapSizes[3](0) + submapSizes[1](0) > submapSizes[2](0) + submapSizes[0](0)) ?
          submapSizes[3](0) + submapSizes[1](0) : submapSizes[2](0) + submapSizes[0](0);
  submapBufferSize[1] =
      (submapSizes[3](1) + submapSizes[2](1) > submapSizes[1](1) + submapSizes[0](1)) ?
          submapSizes[3](1) + submapSizes[2](1) : submapSizes[1](1) + submapSizes[0](1);

  submap.resizeBuffer(submapBufferSize);

  for (auto& mapData : mapFloatData_)
  {
    submap.mapFloatData_[mapData.first].topLeftCorner    (submapSizes[0](0), submapSizes[0](1)) = mapData.second.block(submapIndeces[0](0), submapIndeces[0](1), submapSizes[0](0), submapSizes[0](1));
    submap.mapFloatData_[mapData.first].topRightCorner   (submapSizes[1](0), submapSizes[1](1)) = mapData.second.block(submapIndeces[1](0), submapIndeces[1](1), submapSizes[1](0), submapSizes[1](1));
    submap.mapFloatData_[mapData.first].bottomLeftCorner (submapSizes[2](0), submapSizes[2](1)) = mapData.second.block(submapIndeces[2](0), submapIndeces[2](1), submapSizes[2](0), submapSizes[2](1));
    submap.mapFloatData_[mapData.first].bottomRightCorner(submapSizes[3](0), submapSizes[3](1)) = mapData.second.block(submapIndeces[3](0), submapIndeces[3](1), submapSizes[3](0), submapSizes[3](1));
  }

  for (auto& mapData : mapIntData_)
  {
    submap.mapIntData_[mapData.first].topLeftCorner    (submapSizes[0](0), submapSizes[0](1)) = mapData.second.block(submapIndeces[0](0), submapIndeces[0](1), submapSizes[0](0), submapSizes[0](1));
    submap.mapIntData_[mapData.first].topRightCorner   (submapSizes[1](0), submapSizes[1](1)) = mapData.second.block(submapIndeces[1](0), submapIndeces[1](1), submapSizes[1](0), submapSizes[1](1));
    submap.mapIntData_[mapData.first].bottomLeftCorner (submapSizes[2](0), submapSizes[2](1)) = mapData.second.block(submapIndeces[2](0), submapIndeces[2](1), submapSizes[2](0), submapSizes[2](1));
    submap.mapIntData_[mapData.first].bottomRightCorner(submapSizes[3](0), submapSizes[3](1)) = mapData.second.block(submapIndeces[3](0), submapIndeces[3](1), submapSizes[3](0), submapSizes[3](1));
  }

  // Copy other header information.
  submap.setFrame(frameId_, pose_);
  submap.setTimeStamp(timeStamp_);
  submap.resolution_ = resolution_;

  return submap;
}

const ros::Time& ElevationMapBase::getTimeStamp()
{
  return timeStamp_;
}

void ElevationMapBase::setTimeStamp(ros::Time& timeStamp)
{
  timeStamp_ = timeStamp;
}

const Eigen::Array2d& ElevationMapBase::getLength()
{
  return length_;
}

const kindr::phys_quant::eigen_impl::Position3D& ElevationMapBase::getPosition()
{
  return position_;
}

double ElevationMapBase::getResolution()
{
  return resolution_;
}

const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& ElevationMapBase::getPose()
{
  return pose_;
}

const Eigen::Array2i& ElevationMapBase::getBufferSize()
{
  return bufferSize_;
}

const Eigen::Array2i& ElevationMapBase::getBufferStartIndex()
{
  return bufferStartIndex_;
}

void ElevationMapBase::resetAll()
{
  for (auto& mapData : mapFloatData_)
  {
    mapData.second.setConstant(mapFloatResetValues_[mapData.first]);
  }

  for (auto& mapData : mapIntData_)
  {
    mapData.second.setConstant(mapIntResetValues_[mapData.first]);
  }
}

void ElevationMapBase::resetCols(unsigned int index, unsigned int nCols)
{
  mapFloatData_[FloatDataType::Elevation].block(index, 0, nCols, getBufferSize()[1]).setConstant(mapFloatResetValues_[FloatDataType::Elevation]);
}

void ElevationMapBase::resetRows(unsigned int index, unsigned int nRows)
{
  mapFloatData_[FloatDataType::Elevation].block(0, index, getBufferSize()[0], nRows).setConstant(mapFloatResetValues_[FloatDataType::Elevation]);
}

void ElevationMapBase::resizeBuffer(const Eigen::Array2i& bufferSize)
{
  bufferSize_ = bufferSize;

  for (auto& mapData : mapFloatData_)
  {
    mapData.second.resize(bufferSize_(0), bufferSize_(1));
  }

  for (auto& mapData : mapIntData_)
  {
    mapData.second.resize(bufferSize_(0), bufferSize_(1));
  }
}


} /* namespace elevation_map_commons */
