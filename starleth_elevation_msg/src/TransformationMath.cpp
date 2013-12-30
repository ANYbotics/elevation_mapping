/*
 * TransformationMath.cpp
 *
 *  Created on: Dec 2, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "TransformationMath.hpp"

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

inline Eigen::Matrix2i getMatrixDataOrderToMapFrameTransformation()
{
  return -Matrix2i::Identity();
}

inline Eigen::Matrix2i getMapFrameToMatrixDataOrderTransformation()
{
  return getMatrixDataOrderToMapFrameTransformation().transpose();
}

inline Eigen::Vector2d getIndexVectorFromIndexArray(const Eigen::Array2i& index)
{
  return (getMatrixDataOrderToMapFrameTransformation() * index.matrix()).cast<double>();
}

inline Eigen::Array2i getIndexArrayFromIndexVector(const Eigen::Vector2d& indexVector)
{
  return (getMapFrameToMatrixDataOrderTransformation() * indexVector.cast<int>()).array();
}

bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Array2i& index,
                          const Eigen::Array2d& mapLength,
                          const double& resolution)
{
  Vector2d offset;
  getDistanceOfFirstCell(offset, mapLength, resolution);
  position = offset + resolution * getIndexVectorFromIndexArray(index);
  return true;
}

bool getIndexFromPosition(Eigen::Array2i& index,
                          const Eigen::Vector2d& position,
                          const Eigen::Array2d& mapLength,
                          const double& resolution)
{
  if (!checkIfPositionWithinMap(position, mapLength)) return false;
  Vector2d offset;
  getDistanceOfOrigin(offset, mapLength);
  Vector2d indexVector = ((position - offset).array() / resolution).matrix();
  index = getIndexArrayFromIndexVector(indexVector);
  return true;
}

bool checkIfPositionWithinMap(const Eigen::Vector2d& position,
                              const Eigen::Array2d& mapLength)
{
  Vector2d offset;
  getDistanceOfOrigin(offset, mapLength);
  Vector2d positionTransformed = getMapFrameToMatrixDataOrderTransformation().cast<double>() * (position - offset);

  if(positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0
     && positionTransformed.x() <= mapLength(0) && positionTransformed.y() <= mapLength(1))
  {
    return true;
  }
  return false;
}

}  // namespace
