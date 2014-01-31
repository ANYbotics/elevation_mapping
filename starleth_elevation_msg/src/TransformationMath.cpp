/*
 * TransformationMath.cpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "TransformationMath.hpp"

// fmod
#include <cmath>

using namespace Eigen;

namespace starleth_elevation_msg {

inline bool getDistanceOfOrigin(Eigen::Vector2d& distance,
                                const Eigen::Array2d& mapLength)
{
  distance = (0.5 * mapLength).matrix();
  return true;
}

/*!
 * Gets the distance from the center of the map for the center
 * of the first cell of the map data.
 * @param [out] distance the offset of the center of the cell to the center of the map.
 * @param [in] mapLength the lengths in x and y direction.
 * @param [in] resolution the resolution of the map.
 * @return true if successful.
 */
inline bool getDistanceOfFirstCell(Eigen::Vector2d& distance,
                                   const Eigen::Array2d& mapLength,
                                   const double& resolution)
{
  Eigen::Vector2d distanceOfOrigin;
  getDistanceOfOrigin(distanceOfOrigin, mapLength);

  // Distance of center of cell
  distance = (distanceOfOrigin.array() - 0.5 * resolution).matrix();
  return true;
}

inline Eigen::Matrix2i getBufferOrderToMapFrameTransformation()
{
  return -Matrix2i::Identity();
}

inline Eigen::Matrix2i getMapFrameToBufferOrderTransformation()
{
  return getBufferOrderToMapFrameTransformation().transpose();
}

Eigen::Matrix2i getBufferOrderToMapFrameAlignment()
{
  return getBufferOrderToMapFrameTransformation().array().abs().matrix();
}

inline bool checkIfStartIndexAtDefaultPosition(const Eigen::Array2i& bufferStartIndex)
{
  return ((bufferStartIndex == 0).all());
}

inline Eigen::Array2i getBufferIndexFromIndex(
    const Eigen::Array2i& index,
    const Eigen::Array2i& bufferSize,
    const Eigen::Array2i& bufferStartIndex)
{
  if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
    return index;

  Array2i bufferIndex = index + bufferStartIndex;
  mapIndexWithinRange(bufferIndex, bufferSize);
  return bufferIndex;
}

inline Eigen::Array2i getIndexFromBufferIndex(
    const Eigen::Array2i& bufferIndex,
    const Eigen::Array2i& bufferSize,
    const Eigen::Array2i& bufferStartIndex)
{
  if (checkIfStartIndexAtDefaultPosition(bufferStartIndex))
    return bufferIndex;

  Array2i index = bufferIndex - bufferStartIndex;
  mapIndexWithinRange(index, bufferSize);
  return index;
}

inline Eigen::Vector2d getIndexVectorFromIndex(
    const Eigen::Array2i& index,
    const Eigen::Array2i& bufferSize,
    const Eigen::Array2i& bufferStartIndex)
{
  Array2i unwrappedIndex;
  unwrappedIndex = getIndexFromBufferIndex(index, bufferSize, bufferStartIndex);
  return (getBufferOrderToMapFrameTransformation() * unwrappedIndex.matrix()).cast<double>();
}

inline Eigen::Array2i getIndexFromIndexVector(
    const Eigen::Vector2d& indexVector,
    const Eigen::Array2i& bufferSize,
    const Eigen::Array2i& bufferStartIndex)
{
  Eigen::Array2i index = (getMapFrameToBufferOrderTransformation() * indexVector.cast<int>()).array();
  return getBufferIndexFromIndex(index, bufferSize, bufferStartIndex);
}

bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Array2i& index,
                          const Eigen::Array2d& mapLength,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex)
{
  Vector2d offset;
  getDistanceOfFirstCell(offset, mapLength, resolution);
  position = offset + resolution * getIndexVectorFromIndex(index, bufferSize, bufferStartIndex);
  return true;
}

bool getIndexFromPosition(Eigen::Array2i& index,
                          const Eigen::Vector2d& position,
                          const Eigen::Array2d& mapLength,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex)
{
  if (!checkIfPositionWithinMap(position, mapLength)) return false;
  Vector2d offset;
  getDistanceOfOrigin(offset, mapLength);
  Vector2d indexVector = ((position - offset).array() / resolution).matrix();
  index = getIndexFromIndexVector(indexVector, bufferSize, bufferStartIndex);
  return true;
}

bool checkIfPositionWithinMap(const Eigen::Vector2d& position,
                              const Eigen::Array2d& mapLength)
{
  Vector2d offset;
  getDistanceOfOrigin(offset, mapLength);
  Vector2d positionTransformed = getMapFrameToBufferOrderTransformation().cast<double>() * (position - offset);

  if(positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0
     && positionTransformed.x() <= mapLength(0) && positionTransformed.y() <= mapLength(1))
  {
    return true;
  }
  return false;
}

bool getAlignedPosition(const Eigen::Vector2d& position,
                        Eigen::Vector2d& alignedPosition,
                        const Eigen::Array2d& mapLength,
                        const double& resolution)
{
  Vector2d offset;
  getDistanceOfOrigin(offset, mapLength);

  for (int i = 0; i < position.size(); i++)
  {
    alignedPosition[i] = position[i] - fmod(position[i] - offset[i], resolution);
  }

  return true;
}

bool getIndexShiftFromPositionShift(Eigen::Array2i& indexShift,
                                    const Eigen::Vector2d& positionShift,
                                    const double& resolution)
{
  Vector2d indexShiftVectorTemp = (positionShift.array() / resolution).matrix();
  Vector2i indexShiftVector;

  for (int i = 0; i < indexShiftVector.size(); i++)
  {
    indexShiftVector[i] = static_cast<int>(indexShiftVectorTemp[i] + 0.5 * (indexShiftVectorTemp[i] > 0 ? 1 : -1));
  }

  indexShift = (getMapFrameToBufferOrderTransformation() * indexShiftVector).array();
  return true;
}

void mapIndexWithinRange(Eigen::Array2i& index,
                         const Eigen::Array2i& bufferSize)
{
  for (int i = 0; i < index.size(); i++)
  {
    mapIndexWithinRange(index[i], bufferSize[i]);
  }
}

void mapIndexWithinRange(int& index, const int& bufferSize)
{
  if (index < 0) index += ((-index / bufferSize) + 1) * bufferSize;
  index = index % bufferSize;
}

bool getSubmapIndexAndSize(Eigen::Array2i& submapTopLeftIndex,
                           Eigen::Array2i& centerIndexInSubmap,
                           Eigen::Array2i& submapSize,
                           const Eigen::Vector2d& submapCenter,
                           const Eigen::Array2d& submapLength,
                           const Eigen::Array2d& mapLength,
                           const double& resolution,
                           const Eigen::Array2i& bufferSize,
                           const Eigen::Array2i& bufferStartIndex)
{
  // Get center index
  Array2i centerBufferIndex;
  if (!getIndexFromPosition(centerBufferIndex, submapCenter, mapLength, resolution, bufferSize, bufferStartIndex)) return false;
  Array2i centerRegularIndex = getIndexFromBufferIndex(centerBufferIndex, bufferSize, bufferStartIndex);

  // Get size in cells
  Array2i halfSize;
  for (int i = 0; i < submapLength.size(); i++)
  {
    halfSize[i] = static_cast<int>(ceil(submapLength[i] / resolution / 2.0));
  }

  // Get top left and right bottom cell index
  Array2i topLeftIndex = centerRegularIndex - halfSize;
  Array2i rightBottomIndex = centerRegularIndex + halfSize;

  // Correct to valid indeces
  topLeftIndex = topLeftIndex.max(Array2i::Zero());
  rightBottomIndex = rightBottomIndex.min(bufferSize - Array2i::Ones());

  // Prepare output
  centerIndexInSubmap = centerRegularIndex - topLeftIndex;
  submapTopLeftIndex = getBufferIndexFromIndex(topLeftIndex, bufferSize, bufferStartIndex);
  submapSize = rightBottomIndex - topLeftIndex + Array2i::Ones();

  return true;
}

int getMapRegion(const Eigen::Array2i& index, const Eigen::Array2i& bufferStartIndex)
{
  // TODO Add enum for regions.

  if (index[0] >= bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return 0; // Top left
  if (index[0] >= bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return 1; // Top right
  if (index[0] <  bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return 2; // Bottom left
  if (index[0] <  bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return 3; // Bottom right

  return -1;
}

bool getBufferRegionsForSubmap(std::vector<Eigen::Array2i>& bufferIndeces,
                               std::vector<Eigen::Array2i>& bufferSizes,
                               const Eigen::Array2i& index,
                               const Eigen::Array2i& size,
                               const Eigen::Array2i& bufferSize,
                               const Eigen::Array2i& bufferStartIndex)
{
  if ((getIndexFromBufferIndex(index, bufferSize, bufferStartIndex) + size > bufferSize).all()) return false;

  unsigned int nRegions = 4;

  bufferIndeces.clear();
  bufferIndeces.resize(nRegions, Array2i::Zero());
  bufferSizes.clear();
  bufferSizes.resize(nRegions, Array2i::Zero());

  Array2i bottomRightIndex = index + size - Array2i::Ones();
  mapIndexWithinRange(bottomRightIndex, bufferSize);

  int mapRegionOfTopLeft = getMapRegion(index, bufferStartIndex);
  int mapRegionOfBottomRight = getMapRegion(bottomRightIndex, bufferStartIndex);

  if (mapRegionOfTopLeft == 0)
  {

    if (mapRegionOfBottomRight == 0)
    {
      bufferIndeces[0] = index;
      bufferSizes[0] = size;
      return true;
    }

    if (mapRegionOfBottomRight == 1)
    {
      bufferIndeces[0] = index;
      bufferSizes[0](0) = size(0);
      bufferSizes[0](1) = bufferSize(1) - index(1);

      bufferIndeces[1](0) = index(0);
      bufferIndeces[1](1) = 0;
      bufferSizes[1](0) = size(0);
      bufferSizes[1](1) = size(1) - bufferSizes[0](1);
      return true;
    }

    if (mapRegionOfBottomRight == 2)
    {
      bufferIndeces[0] = index;
      bufferSizes[0](0) = bufferSize(0) - index(0);
      bufferSizes[0](1) = size(1);

      bufferIndeces[2](0) = 0;
      bufferIndeces[2](1) = index(1);
      bufferSizes[2](0) = size(0) - bufferSizes[0](0);
      bufferSizes[2](1) = size(1);
      return true;
    }

    if (mapRegionOfBottomRight == 3)
    {
      bufferIndeces[0] = index;
      bufferSizes[0](0) = bufferSize(0) - index(0);
      bufferSizes[0](1) = bufferSize(1) - index(1);

      bufferIndeces[1](0) = index(0);
      bufferIndeces[1](1) = 0;
      bufferSizes[1](0) = bufferSize(0) - index(0);
      bufferSizes[1](1) = size(1) - bufferSizes[0](1);

      bufferIndeces[2](0) = 0;
      bufferIndeces[2](1) = index(1);
      bufferSizes[2](0) = size(0) - bufferSizes[0](0);
      bufferSizes[2](1) = bufferSize(1) - index(1);

      bufferIndeces[3] = Array2i::Zero();
      bufferSizes[3](0) = bufferSizes[2](0);
      bufferSizes[3](1) = bufferSizes[1](1);
      return true;
    }

  }
  else if (mapRegionOfTopLeft == 1)
  {

    if (mapRegionOfBottomRight == 1)
    {
      bufferIndeces[1] = index;
      bufferSizes[1] = size;
      return true;
    }

    if (mapRegionOfBottomRight == 3)
    {
      bufferIndeces[1] = index;
      bufferSizes[1](0) = bufferSize(0) - index(0);
      bufferSizes[1](1) = size(1);

      bufferIndeces[3](0) = 0;
      bufferIndeces[3](1) = index(1);
      bufferSizes[3](0) = size(0) - bufferSizes[1](0);
      bufferSizes[3](1) = size(1);
      return true;
    }

  }
  else if (mapRegionOfTopLeft == 2)
  {

    if (mapRegionOfBottomRight == 2)
    {
      bufferIndeces[2] = index;
      bufferSizes[2] = size;
      return true;
    }

    if (mapRegionOfBottomRight == 3)
    {
      bufferIndeces[2] = index;
      bufferSizes[2](0) = size(0);
      bufferSizes[2](1) = bufferSize(1) - index(1);

      bufferIndeces[3](0) = index(0);
      bufferIndeces[3](1) = 0;
      bufferSizes[3](0) = size(0);
      bufferSizes[3](1) = size(1) - bufferSizes[2](1);
      return true;
    }

  }
  else if (mapRegionOfTopLeft == 3)
  {

    if (mapRegionOfBottomRight == 3)
    {
      bufferIndeces[3] = index;
      bufferSizes[3] = size;
      return true;
    }

  }

  return false;
}

}  // namespace
