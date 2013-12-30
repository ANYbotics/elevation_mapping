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

/*!
 * Gets the distance from the center of the map for the center
 * of the first cell of the map data.
 * @param [out] distance the offset of the center of the cell to the center of the map.
 * @param [in] mapLength the lengths in x and y direction.
 * @param [in] resolution the resolution of the map.
 * @return true if successful.
 */

inline bool getDistanceOfFirstCellFromCenter(Eigen::Vector2d& distance,
                                      const Eigen::Array2d& mapLength,
                                      const double& resolution)
{
  // Distance of center of cell
  distance = (0.5 * mapLength - 0.5 * resolution).matrix();
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

inline Eigen::Array2i getIndexArrayFromIndexVector(const Eigen::Vector2d& index)
{
  return (getMapFrameToMatrixDataOrderTransformation() * index.cast<int>()).array();
}

bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Array2i& index,
                          const Eigen::Array2d& mapLength,
                          const double& resolution)
{
  Vector2d offset;
  getDistanceOfFirstCellFromCenter(offset, mapLength, resolution);
  position = offset + resolution * getIndexVectorFromIndexArray(index);
  return true;
}

bool getIndexFromPosition(Eigen::Array2i& index,
                          const Eigen::Vector2d& position,
                          const Eigen::Array2d& mapLength,
                          const double& resolution)
{
  if (!checkIfPositionWithinMap(position, mapLength, resolution)) return false;
  Vector2d offset;
  getDistanceOfFirstCellFromCenter(offset, mapLength, resolution);
  Vector2d indexVector = ((position - offset).array() / resolution).matrix();
  index = getIndexArrayFromIndexVector(indexVector);
  return true;
}

bool checkIfPositionWithinMap(const Eigen::Vector2d& position,
                              const Eigen::Array2d& mapLength,
                              const double& resolution)
{
  Vector2d offset;
  getDistanceOfFirstCellFromCenter(offset, mapLength, resolution);
  Vector2d positionTransformed = getMapFrameToMatrixDataOrderTransformation().cast<double>() * (position - offset);

  if(positionTransformed.x() >= 0.0 && positionTransformed.y() >= 0.0
     && positionTransformed.x() <= mapLength(0) && positionTransformed.y() <= mapLength(1))
  {
    return true;
  }
  return false;
}

}  // namespace
