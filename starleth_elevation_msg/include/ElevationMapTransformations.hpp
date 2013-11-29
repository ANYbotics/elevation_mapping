/*
 * ElevationMapTransformations.hpp
 *
 *  Created on: Nov 27, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <starleth_elevation_msg/ElevationMap.h>

// Eigen
#include <Eigen/Core>

using namespace Eigen;

namespace starleth_elevation_msg {

////! Converts an Eigen matrix into a Float64MultiArray message
//template <class Derived>
//void matrixEigenToMsg(const Eigen::MatrixBase<Derived>& e, std_msgs::Float64MultiArray& m)
//{
//  if (m.layout.dim.size() != 2)
//    m.layout.dim.resize(2);
//  m.layout.dim[0].stride = e.rows() * e.cols();
//  m.layout.dim[0].size = e.rows();
//  m.layout.dim[1].stride = e.cols();
//  m.layout.dim[1].size = e.cols();
//  if ((int)m.data.size() != e.size())
//    m.data.resize(e.size());
//  int ii = 0;
//  for (int i = 0; i < e.rows(); ++i)
//    for (int j = 0; j < e.cols(); ++j)
//      m.data[ii++] = e.coeff(i, j);
//}

inline void getDistanceOfFirstCellFromCenter(Eigen::Vector2d& distance, const starleth_elevation_msg::ElevationMap& map)
{
  // Distance of center of cell
  distance.x() = -0.5 * map.lengthInX + 0.5 * map.resolution;
  distance.y() =  0.5 * map.lengthInY - 0.5 * map.resolution;
}

inline Matrix2i getIndexToPositionDirectionTransformation()
{
  Matrix2i transformation;
  transformation << 0, 1, -1, 0;
  return transformation;
}

/*!
 * Returns the 1d array index to access the map data based on the 2d matrix indeces
 * @param index the 2d matrix indeces in column major format
 * @param map the reference to the map
 * @return the 1d array index
 */
inline unsigned int get1dIndexFrom2dIndex(
    const Eigen::Vector2i& index,
    const starleth_elevation_msg::ElevationMap& map)
{
  unsigned int n = map.elevation.layout.data_offset + index(0) * map.elevation.layout.dim[1].stride + index(1);
  return n;
}

/*!
 * Returns the 2d matrix indeces based on the 1d array index
 * @param n the 1d array index
 * @param map the reference to the map
 * @return 2d matrix indeces in column major format
 */
inline Eigen::Vector2i get2dIndexFrom1dIndex(
    unsigned int n, const starleth_elevation_msg::ElevationMap& map)
{
  Eigen::Vector2i index;
  index(1) = n - map.elevation.layout.data_offset % map.elevation.layout.dim[1].stride;
  index(0) = (int)((n - map.elevation.layout.data_offset - index(1)) / map.elevation.layout.dim[1].stride);
  return index;
}

/*!
 * Gets the position of the center of the cell in the map coordinate system
 * from the index of the cell
 * @param [out] position the center of the cell in the map coordinate system (2d)
 * @param [in] index of the cell
 * @param [in] map the reference to the map
 * @return true
 */
bool getPositionFromIndex(Eigen::Vector2d& position,
                          const Eigen::Vector2i& index,
                          const starleth_elevation_msg::ElevationMap& map)
{
  Vector2d offset;
  getDistanceOfFirstCellFromCenter(offset, map);
  position = offset + map.resolution *
            (getIndexToPositionDirectionTransformation() * index).cast<double>();
  return true;
}

} // namespace
