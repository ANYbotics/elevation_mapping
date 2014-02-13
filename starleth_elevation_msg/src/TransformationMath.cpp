/*
 * TransformationMath.cpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "TransformationMath.hpp"

// STD
#include <map>

// fmod
#include <cmath>

using namespace Eigen;

namespace starleth_elevation_msg {

namespace internal {

enum class bufferRegion : int
{
    TopLeft = 0,
    TopRight = 1,
    BottomLeft = 2,
    BottomRight = 3
};

unsigned int nBufferRegions = 4;

std::map<bufferRegion, int> bufferRegionIndeces =
{
{ bufferRegion::TopLeft, 0 },
{ bufferRegion::TopRight, 1 },
{ bufferRegion::BottomLeft, 2 },
{ bufferRegion::BottomRight, 3 } };

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

inline bool checkIfIndexWithinRange(const Eigen::Array2i& index, const Eigen::Array2i& bufferSize)
{
  if (index[0] >= 0 && index[1] >= 0.0 && index[0] < bufferSize[0] && index[1] < bufferSize[1])
  {
    return true;
  }
  return false;
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

inline bufferRegion getMapRegion(const Eigen::Array2i& index, const Eigen::Array2i& bufferStartIndex)
{
  if (index[0] >= bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return bufferRegion::TopLeft;
  if (index[0] >= bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return bufferRegion::TopRight;
  if (index[0] <  bufferStartIndex[0] && index[1] >= bufferStartIndex[1]) return bufferRegion::BottomLeft;
  if (index[0] <  bufferStartIndex[0] && index[1] <  bufferStartIndex[1]) return bufferRegion::BottomRight;
}

} // namespace

using namespace internal;

bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Array2i& index,
                          const Eigen::Array2d& mapLength,
                          const double& resolution,
                          const Eigen::Array2i& bufferSize,
                          const Eigen::Array2i& bufferStartIndex)
{
  if (!checkIfIndexWithinRange(index, bufferSize)) return false;
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

bool getPositionShiftFromIndexShift(Eigen::Vector2d& positionShift,
                                    const Eigen::Array2i& indexShift,
                                    const double& resolution)
{
  positionShift = (getBufferOrderToMapFrameTransformation() * indexShift.matrix()).cast<double>() * resolution;
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
  centerIndexInSubmap = centerRegularIndex - topLeftIndex; // This is a submap index, so no "getBufferIndexFromIndex" necessary
  submapTopLeftIndex = getBufferIndexFromIndex(topLeftIndex, bufferSize, bufferStartIndex);
  submapSize = rightBottomIndex - topLeftIndex + Array2i::Ones();

  return true;
}

bool getBufferRegionsForSubmap(std::vector<Eigen::Array2i>& bufferIndeces,
                               std::vector<Eigen::Array2i>& bufferSizes,
                               const Eigen::Array2i& bufferIndex,
                               const Eigen::Array2i& size,
                               const Eigen::Array2i& bufferSize,
                               const Eigen::Array2i& bufferStartIndex)
{
  if ((getIndexFromBufferIndex(bufferIndex, bufferSize, bufferStartIndex) + size > bufferSize).all()) return false;

  bufferIndeces.clear();
  bufferIndeces.resize(nBufferRegions, Array2i::Zero());
  bufferSizes.clear();
  bufferSizes.resize(nBufferRegions, Array2i::Zero());

  Array2i bottomRightIndex = bufferIndex + size - Array2i::Ones();
  mapIndexWithinRange(bottomRightIndex, bufferSize);

  bufferRegion mapRegionOfTopLeft = getMapRegion(bufferIndex, bufferStartIndex);
  bufferRegion mapRegionOfBottomRight = getMapRegion(bottomRightIndex, bufferStartIndex);

  unsigned int topLeft = bufferRegionIndeces[bufferRegion::TopLeft];
  unsigned int topRight = bufferRegionIndeces[bufferRegion::TopRight];
  unsigned int bottomLeft= bufferRegionIndeces[bufferRegion::BottomLeft];
  unsigned int bottomRight= bufferRegionIndeces[bufferRegion::BottomRight];

  if (mapRegionOfTopLeft == bufferRegion::TopLeft)
  {

    if (mapRegionOfBottomRight == bufferRegion::TopLeft)
    {
      bufferIndeces[topLeft] = bufferIndex;
      bufferSizes[topLeft] = size;
      return true;
    }

    if (mapRegionOfBottomRight == bufferRegion::TopRight)
    {
      bufferIndeces[topLeft] = bufferIndex;
      bufferSizes[topLeft](0) = size(0);
      bufferSizes[topLeft](1) = bufferSize(1) - bufferIndex(1);

      bufferIndeces[topRight](0) = bufferIndex(0);
      bufferIndeces[topRight](1) = 0;
      bufferSizes[topRight](0) = size(0);
      bufferSizes[topRight](1) = size(1) - bufferSizes[topLeft](1);
      return true;
    }

    if (mapRegionOfBottomRight == bufferRegion::BottomLeft)
    {
      bufferIndeces[topLeft] = bufferIndex;
      bufferSizes[topLeft](0) = bufferSize(0) - bufferIndex(0);
      bufferSizes[topLeft](1) = size(1);

      bufferIndeces[bottomLeft](0) = 0;
      bufferIndeces[bottomLeft](1) = bufferIndex(1);
      bufferSizes[bottomLeft](0) = size(0) - bufferSizes[topLeft](0);
      bufferSizes[bottomLeft](1) = size(1);
      return true;
    }

    if (mapRegionOfBottomRight == bufferRegion::BottomRight)
    {
      bufferIndeces[topLeft] = bufferIndex;
      bufferSizes[topLeft](0) = bufferSize(0) - bufferIndex(0);
      bufferSizes[topLeft](1) = bufferSize(1) - bufferIndex(1);

      bufferIndeces[topRight](0) = bufferIndex(0);
      bufferIndeces[topRight](1) = 0;
      bufferSizes[topRight](0) = bufferSize(0) - bufferIndex(0);
      bufferSizes[topRight](1) = size(1) - bufferSizes[topLeft](1);

      bufferIndeces[bottomLeft](0) = 0;
      bufferIndeces[bottomLeft](1) = bufferIndex(1);
      bufferSizes[bottomLeft](0) = size(0) - bufferSizes[topLeft](0);
      bufferSizes[bottomLeft](1) = bufferSize(1) - bufferIndex(1);

      bufferIndeces[bottomRight] = Array2i::Zero();
      bufferSizes[bottomRight](0) = bufferSizes[bottomLeft](0);
      bufferSizes[bottomRight](1) = bufferSizes[topRight](1);
      return true;
    }

  }
  else if (mapRegionOfTopLeft == bufferRegion::TopRight)
  {

    if (mapRegionOfBottomRight == bufferRegion::TopRight)
    {
      bufferIndeces[topRight] = bufferIndex;
      bufferSizes[topRight] = size;
      return true;
    }

    if (mapRegionOfBottomRight == bufferRegion::BottomRight)
    {
      bufferIndeces[topRight] = bufferIndex;
      bufferSizes[topRight](0) = bufferSize(0) - bufferIndex(0);
      bufferSizes[topRight](1) = size(1);

      bufferIndeces[bottomRight](0) = 0;
      bufferIndeces[bottomRight](1) = bufferIndex(1);
      bufferSizes[bottomRight](0) = size(0) - bufferSizes[topRight](0);
      bufferSizes[bottomRight](1) = size(1);
      return true;
    }

  }
  else if (mapRegionOfTopLeft == bufferRegion::BottomLeft)
  {

    if (mapRegionOfBottomRight == bufferRegion::BottomLeft)
    {
      bufferIndeces[bottomLeft] = bufferIndex;
      bufferSizes[bottomLeft] = size;
      return true;
    }

    if (mapRegionOfBottomRight == bufferRegion::BottomRight)
    {
      bufferIndeces[bottomLeft] = bufferIndex;
      bufferSizes[bottomLeft](0) = size(0);
      bufferSizes[bottomLeft](1) = bufferSize(1) - bufferIndex(1);

      bufferIndeces[bottomRight](0) = bufferIndex(0);
      bufferIndeces[bottomRight](1) = 0;
      bufferSizes[bottomRight](0) = size(0);
      bufferSizes[bottomRight](1) = size(1) - bufferSizes[bottomLeft](1);
      return true;
    }

  }
  else if (mapRegionOfTopLeft == bufferRegion::BottomRight)
  {

    if (mapRegionOfBottomRight == bufferRegion::BottomRight)
    {
      bufferIndeces[bottomRight] = bufferIndex;
      bufferSizes[bottomRight] = size;
      return true;
    }

  }

  return false;
}


Eigen::Matrix2i getBufferOrderToMapFrameAlignment()
{
  return getBufferOrderToMapFrameTransformation().cwiseAbs();
}

}  // namespace
