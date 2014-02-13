/*
 * TransformationMathTest.cpp
 *
 *  Created on: Feb 10, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "TransformationMath.hpp"

// Eigen
#include <Eigen/Core>

// gtest
#include <gtest/gtest.h>

// Limits
#include <cfloat>

using namespace std;
using namespace Eigen;
using namespace starleth_elevation_msg;

TEST(PositionFromIndex, Simple)
{
  Array2d mapLength(3.0, 2.0);
  double resolution = 1.0;
  Array2i bufferSize(3, 2);
  Vector2d position;

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(0, 0), mapLength, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(1.0, position.x());
  EXPECT_DOUBLE_EQ(0.5, position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(1, 0), mapLength, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.5, position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(1, 1), mapLength, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(-0.5, position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(2, 1), mapLength, resolution, bufferSize));
  EXPECT_DOUBLE_EQ(-1.0, position.x());
  EXPECT_DOUBLE_EQ(-0.5, position.y());

  EXPECT_FALSE(getPositionFromIndex(position, Array2i(3, 1), mapLength, resolution, bufferSize));
}

TEST(PositionFromIndex, CircularBuffer)
{
  Array2d mapLength(0.5, 0.4);
  double resolution = 0.1;
  Array2i bufferSize(5, 4);
  Array2i bufferStartIndex(3, 1);
  Vector2d position;

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(3, 1), mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.2, position.x());
  EXPECT_DOUBLE_EQ(0.15, position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(4, 2), mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.1, position.x());
  EXPECT_DOUBLE_EQ(0.05, position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(2, 0), mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(-0.2, position.x());
  EXPECT_DOUBLE_EQ(-0.15, position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(0, 0), mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(-0.15, position.y());

  EXPECT_TRUE(getPositionFromIndex(position, Array2i(4, 3), mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_DOUBLE_EQ(0.1, position.x());
  EXPECT_DOUBLE_EQ(-0.05, position.y());

  EXPECT_FALSE(getPositionFromIndex(position, Array2i(5, 3), mapLength, resolution, bufferSize, bufferStartIndex));
}

TEST(IndexFromPosition, Simple)
{
  Array2d mapLength(3.0, 2.0);
  double resolution = 1.0;
  Array2i bufferSize(3, 2);
  Array2i index;

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(1.0, 0.5), mapLength, resolution, bufferSize));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(-1.0, -0.5), mapLength, resolution, bufferSize));
  EXPECT_EQ(2, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.6, 0.1), mapLength, resolution, bufferSize));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.4, -0.1), mapLength, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.4, 0.1), mapLength, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_FALSE(getIndexFromPosition(index, Vector2d(4.0, 0.5), mapLength, resolution, bufferSize));
}

TEST(IndexFromPosition, EdgeCases)
{
  Array2d mapLength(3.0, 2.0);
  double resolution = 1.0;
  Array2i bufferSize(3, 2);
  Array2i index;

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.0, DBL_EPSILON), mapLength, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(0, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.5 - DBL_EPSILON, -DBL_EPSILON), mapLength, resolution, bufferSize));
  EXPECT_EQ(1, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(-0.5 - DBL_EPSILON, -DBL_EPSILON), mapLength, resolution, bufferSize));
  EXPECT_EQ(2, index(0));
  EXPECT_EQ(1, index(1));
}

TEST(IndexFromPosition, CircularBuffer)
{
  Array2d mapLength(0.5, 0.4);
  double resolution = 0.1;
  Array2i bufferSize(5, 4);
  Array2i bufferStartIndex(3, 1);
  Array2i index;

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.2, 0.15), mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, index(0));
  EXPECT_EQ(1, index(1));

  EXPECT_TRUE(getIndexFromPosition(index, Vector2d(0.03, -0.17), mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, index(0));
  EXPECT_EQ(0, index(1));
}

TEST(checkIfPositionWithinMap, Inside)
{
  Array2d mapLength(50.0, 25.0);

  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(0.0, 0.0), mapLength));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(5.0, 5.0), mapLength));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(20.0, 10.0), mapLength));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(20.0, -10.0), mapLength));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(-20.0, 10.0), mapLength));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(-20.0, -10.0), mapLength));
}

TEST(checkIfPositionWithinMap, Outside)
{
  Array2d mapLength(10.0, 5.0);

  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(5.5, 0.0), mapLength));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(-5.5, 0.0), mapLength));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(-5.5, 3.0), mapLength));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(-5.5, -3.0), mapLength));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(3.0, 3.0), mapLength));
}

TEST(checkIfPositionWithinMap, EdgeCases)
{
  Array2d mapLength(2.0, 3.0);

  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(1.0, -1.5), mapLength));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(-1.0, 1.5), mapLength));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(1.0 + DBL_EPSILON, 1.0), mapLength));
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d((2.0 + DBL_EPSILON) / 2.0, 1.0), mapLength));
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(0.5, -1.5 - 2.0 * DBL_EPSILON), mapLength)); // TODO Why is factor 2.0 necessary?
  EXPECT_TRUE(checkIfPositionWithinMap(Vector2d(-0.5, (3.0 + DBL_EPSILON) / 2.0), mapLength));
}

TEST(getIndexShiftFromPositionShift, All)
{
  double resolution = 1.0;
  Array2i indexShift;

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(0.0, 0.0), resolution));
  EXPECT_EQ(0, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(0.35, -0.45), resolution));
  EXPECT_EQ(0, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(0.55, -0.45), resolution));
  EXPECT_EQ(-1, indexShift(0));
  EXPECT_EQ(0, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(-1.3, -2.65), resolution));
  EXPECT_EQ(1, indexShift(0));
  EXPECT_EQ(3, indexShift(1));

  EXPECT_TRUE(getIndexShiftFromPositionShift(indexShift, Vector2d(-0.4, 0.09), 0.2));
  EXPECT_EQ(2, indexShift(0));
  EXPECT_EQ(0, indexShift(1));
}

TEST(getPositionShiftFromIndexShift, All)
{
  double resolution = 0.3;
  Vector2d positionShift;

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Array2i(0, 0), resolution));
  EXPECT_DOUBLE_EQ(0.0, positionShift.x());
  EXPECT_DOUBLE_EQ(0.0, positionShift.y());

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Array2i(1, -1), resolution));
  EXPECT_DOUBLE_EQ(-0.3, positionShift.x());
  EXPECT_DOUBLE_EQ(0.3, positionShift.y());

  EXPECT_TRUE(getPositionShiftFromIndexShift(positionShift, Array2i(2, 1), resolution));
  EXPECT_DOUBLE_EQ(-0.6, positionShift.x());
  EXPECT_DOUBLE_EQ(-0.3, positionShift.y());
}

TEST(checkIfIndexWithinRange, All)
{
  Array2i bufferSize(10, 15);
  EXPECT_TRUE(checkIfIndexWithinRange(Array2i(0, 0), bufferSize));
  EXPECT_TRUE(checkIfIndexWithinRange(Array2i(9, 14), bufferSize));
  EXPECT_FALSE(checkIfIndexWithinRange(Array2i(10, 5), bufferSize));
  EXPECT_FALSE(checkIfIndexWithinRange(Array2i(5, 300), bufferSize));
  EXPECT_FALSE(checkIfIndexWithinRange(Array2i(-1, 0), bufferSize));
  EXPECT_FALSE(checkIfIndexWithinRange(Array2i(0, -300), bufferSize));
}

//TEST(limitIndexToRange, Simple)
//{
//  Array2i bufferSize(10, 15);
//  Array2i index;
//
//  index << 0, 0;
//  limitIndexToRange(index, bufferSize);
//  EXPECT_EQ(0, index(0));
//  EXPECT_EQ(0, index(1));
//
//  index << 9, 14;
//  limitIndexToRange(index, bufferSize);
//  EXPECT_EQ(9, index(0));
//  EXPECT_EQ(14, index(1));
//
//  index << 10, 15;
//  limitIndexToRange(index, bufferSize);
//  EXPECT_EQ(9, index(0));
//  EXPECT_EQ(14, index(1));
//
//  index << -1, -1;
//  limitIndexToRange(index, bufferSize);
//  EXPECT_EQ(0, index(0));
//  EXPECT_EQ(0, index(1));
//
//  index << 1e6, 1e6;
//  limitIndexToRange(index, bufferSize);
//  EXPECT_EQ(9, index(0));
//  EXPECT_EQ(14, index(1));
//
//  index << -1e6, -1e6;
//  limitIndexToRange(index, bufferSize);
//  EXPECT_EQ(0, index(0));
//  EXPECT_EQ(0, index(1));
//}

TEST(mapIndexWithinRange, All)
{
  int index;
  int bufferSize = 10;

  index = 0;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 1;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -1;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 9;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(9, index);

  index = 10;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(0, index);

  index = 11;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = 35;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(5, index);

  index = -9;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(1, index);

  index = -19;
  mapIndexWithinRange(index, bufferSize);
  EXPECT_EQ(1, index);
}

TEST(limitPositionToRange, All)
{
  Array2d mapLength(30.0, 10.0);
  Vector2d position;

  position << 0.0, 0.0;
  limitPositionToRange(position, mapLength);
  EXPECT_DOUBLE_EQ(0.0, position.x());
  EXPECT_DOUBLE_EQ(0.0, position.y());

  position << 15.0, 5.0;
  limitPositionToRange(position, mapLength);
  EXPECT_DOUBLE_EQ(15.0, position.x());
  EXPECT_DOUBLE_EQ(5.0, position.y());

  position << -15.0, -5.0;
  limitPositionToRange(position, mapLength);
  EXPECT_DOUBLE_EQ(-15.0, position.x());
  EXPECT_DOUBLE_EQ(-5.0, position.y());

  position << 16.0, 6.0;
  limitPositionToRange(position, mapLength);
  EXPECT_DOUBLE_EQ(15.0, position.x());
  EXPECT_DOUBLE_EQ(5.0, position.y());

  position << -16.0, -6.0;
  limitPositionToRange(position, mapLength);
  EXPECT_DOUBLE_EQ(-15.0, position.x());
  EXPECT_DOUBLE_EQ(-5.0, position.y());

  position << 1e6, 1e6;
  limitPositionToRange(position, mapLength);
  EXPECT_DOUBLE_EQ(15.0, position.x());
  EXPECT_DOUBLE_EQ(5.0, position.y());

  position << -1e6, -1e6;
  limitPositionToRange(position, mapLength);
  EXPECT_DOUBLE_EQ(-15.0, position.x());
  EXPECT_DOUBLE_EQ(-5.0, position.y());
}
//
//TEST(getSubmapIndexAndSize, Simple)
//{
//  // Map
//  Array2d mapLength(3.0, 2.0);
//  double resolution = 0.1;
//  Array2i bufferSize(30, 20);
//
//  // Requested submap
//  Vector2d submapCenter(0.5, 0.0);
//  Vector2d submapLength(1.0, 1.5);
//
//  // The returned submap indeces
//  Array2i submapTopLeftIndex;
//  Array2i centerIndexInSubmap;
//  Array2i submapSize;
//
//  EXPECT_TRUE(getSubmapIndexAndSize(submapTopLeftIndex, centerIndexInSubmap, submapSize,
//                                    submapCenter, submapLength,
//                                    mapLength, resolution, bufferSize));
//  EXPECT_LE(10, submapSize.x());
//  EXPECT_GE(10 + 2, submapSize.x());
//  EXPECT_LE(15, submapSize.y());
//  EXPECT_GE(15 + 2, submapSize.y());
//}
//
//TEST(getSubmapIndexAndSize, ExceedingBoundaries)
//{
//  // Map
//  Array2d mapLength(3.0, 2.0);
//  double resolution = 0.1;
//  Array2i bufferSize(30, 20);
//
//  // Requested submap
//  Vector2d submapCenter(1.0, 0.0);
//  Vector2d submapLength(1e6, 1e6);
//
//  // The returned submap indeces
//  Array2i submapTopLeftIndex;
//  Array2i centerIndexInSubmap;
//  Array2i submapSize;
//
//  EXPECT_TRUE(getSubmapIndexAndSize(submapTopLeftIndex, centerIndexInSubmap, submapSize,
//                                    submapCenter, submapLength,
//                                    mapLength, resolution, bufferSize));
//  EXPECT_EQ(30, submapSize.x());
//  EXPECT_EQ(20, submapSize.y());
//}
