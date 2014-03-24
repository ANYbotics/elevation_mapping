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

// Vector
#include <vector>

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
  EXPECT_FALSE(checkIfPositionWithinMap(Vector2d(0.5, -1.5 - (2.0 * DBL_EPSILON)), mapLength));
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
  EXPECT_LT(14.9, position.x());
  EXPECT_GT(15.1, position.x());
  EXPECT_LT(4.9, position.y());
  EXPECT_GT(5.1, position.y());

  position << -1e6, -1e6;
  limitPositionToRange(position, mapLength);
  EXPECT_LT(-15.1, position.x());
  EXPECT_GT(-14.9, position.x());
  EXPECT_LT(-5.1, position.y());
  EXPECT_GT(-4.9, position.y());
}

TEST(getSubmapInformation, Simple)
{
  // Map
  Array2d mapLength(5.0, 4.0);
  double resolution = 1.0;
  Array2i bufferSize(5, 4);

  // Requested submap
  Vector2d requestedSubmapPosition;
  Vector2d requestedSubmapLength;

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  requestedSubmapPosition << 0.0, 0.5;
  requestedSubmapLength << 0.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                           requestedSubmapPosition, requestedSubmapLength, mapLength, resolution, bufferSize));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(3, submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.5, submapPosition.y());
  EXPECT_DOUBLE_EQ(1.0, submapLength(0));
  EXPECT_DOUBLE_EQ(3.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(1, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, Zero)
{
  // Map
  Array2d mapLength(5.0, 4.0);
  double resolution = 1.0;
  Array2i bufferSize(5, 4);

  // Requested submap
  Vector2d requestedSubmapPosition;
  Vector2d requestedSubmapLength;

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  requestedSubmapPosition << -1.0, -0.5;
  requestedSubmapLength << 0.0, 0.0;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, resolution, bufferSize));
  EXPECT_EQ(3, submapTopLeftIndex(0));
  EXPECT_EQ(2, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(1, submapSize(1));
  EXPECT_DOUBLE_EQ(requestedSubmapPosition.x(), submapPosition.x());
  EXPECT_DOUBLE_EQ(requestedSubmapPosition.y(), submapPosition.y());
  EXPECT_DOUBLE_EQ(resolution, submapLength(0));
  EXPECT_DOUBLE_EQ(resolution, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, ExceedingBoundaries)
{
  // Map
  Array2d mapLength(5.0, 4.0);
  double resolution = 1.0;
  Array2i bufferSize(5, 4);

  // Requested submap
  Vector2d requestedSubmapPosition;
  Vector2d requestedSubmapLength;

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  requestedSubmapPosition << 2.0, 1.5;
  requestedSubmapLength << 2.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, resolution, bufferSize));
  EXPECT_EQ(0, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(2, submapSize(0));
  EXPECT_EQ(2, submapSize(1));
  EXPECT_DOUBLE_EQ(1.5, submapPosition.x());
  EXPECT_DOUBLE_EQ(1.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(2.0, submapLength(0));
  EXPECT_DOUBLE_EQ(2.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));

  requestedSubmapPosition << 0.0, 0.0;
  requestedSubmapLength << 1e6, 1e6;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, resolution, bufferSize));
  EXPECT_EQ(0, submapTopLeftIndex(0));
  EXPECT_EQ(0, submapTopLeftIndex(1));
  EXPECT_EQ(bufferSize(0), submapSize(0));
  EXPECT_EQ(bufferSize(1), submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(mapLength(0), submapLength(0));
  EXPECT_DOUBLE_EQ(mapLength(1), submapLength(1));
  EXPECT_EQ(2, requestedIndexInSubmap(0));
  EXPECT_LE(1, requestedIndexInSubmap(1));
  EXPECT_GE(2, requestedIndexInSubmap(1));
}

TEST(getSubmapInformation, CircularBuffer)
{
  // Map
  Array2d mapLength(5.0, 4.0);
  double resolution = 1.0;
  Array2i bufferSize(5, 4);
  Array2i bufferStartIndex(2, 1);

  // Requested submap
  Vector2d requestedSubmapPosition;
  Vector2d requestedSubmapLength;

  // The returned submap indeces
  Array2i submapTopLeftIndex;
  Array2i submapSize;
  Eigen::Vector2d submapPosition;
  Eigen::Array2d submapLength;
  Eigen::Array2i requestedIndexInSubmap;

  requestedSubmapPosition << 0.0, 0.5;
  requestedSubmapLength << 0.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(4, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(1, submapSize(0));
  EXPECT_EQ(3, submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.5, submapPosition.y());
  EXPECT_DOUBLE_EQ(1.0, submapLength(0));
  EXPECT_DOUBLE_EQ(3.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(1, requestedIndexInSubmap(1));

  requestedSubmapPosition << 2.0, 1.5;
  requestedSubmapLength << 2.9, 2.9;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(2, submapSize(0));
  EXPECT_EQ(2, submapSize(1));
  EXPECT_DOUBLE_EQ(1.5, submapPosition.x());
  EXPECT_DOUBLE_EQ(1.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(2.0, submapLength(0));
  EXPECT_DOUBLE_EQ(2.0, submapLength(1));
  EXPECT_EQ(0, requestedIndexInSubmap(0));
  EXPECT_EQ(0, requestedIndexInSubmap(1));

  requestedSubmapPosition << 0.0, 0.0;
  requestedSubmapLength << 1e6, 1e6;
  EXPECT_TRUE(getSubmapInformation(submapTopLeftIndex, submapSize, submapPosition, submapLength, requestedIndexInSubmap,
                                   requestedSubmapPosition, requestedSubmapLength,
                                    mapLength, resolution, bufferSize, bufferStartIndex));
  EXPECT_EQ(2, submapTopLeftIndex(0));
  EXPECT_EQ(1, submapTopLeftIndex(1));
  EXPECT_EQ(bufferSize(0), submapSize(0));
  EXPECT_EQ(bufferSize(1), submapSize(1));
  EXPECT_DOUBLE_EQ(0.0, submapPosition.x());
  EXPECT_DOUBLE_EQ(0.0, submapPosition.y());
  EXPECT_DOUBLE_EQ(mapLength(0), submapLength(0));
  EXPECT_DOUBLE_EQ(mapLength(1), submapLength(1));
  EXPECT_EQ(2, requestedIndexInSubmap(0));
  EXPECT_LE(1, requestedIndexInSubmap(1));
  EXPECT_GE(2, requestedIndexInSubmap(1));
}

TEST(getBufferRegionsForSubmap, Trivial)
{
  unsigned int topLeft = bufferRegionIndeces[bufferRegion::TopLeft];
  unsigned int topRight = bufferRegionIndeces[bufferRegion::TopRight];
  unsigned int bottomLeft= bufferRegionIndeces[bufferRegion::BottomLeft];
  unsigned int bottomRight= bufferRegionIndeces[bufferRegion::BottomRight];

  Eigen::Array2i bufferSize(5, 4);
  vector<Eigen::Array2i> submapIndeces, submapSizes;

  Eigen::Array2i submapIndex(0, 0);
  Eigen::Array2i submapSize(0, 0);
  EXPECT_TRUE(getBufferRegionsForSubmap(submapIndeces, submapSizes, submapIndex, submapSize, bufferSize));
  EXPECT_EQ(0, submapSizes[topLeft][0]);
  EXPECT_EQ(0, submapSizes[topLeft][1]);
  EXPECT_EQ(0, submapSizes[topRight][0]);
  EXPECT_EQ(0, submapSizes[topRight][1]);
  EXPECT_EQ(0, submapSizes[bottomLeft][0]);
  EXPECT_EQ(0, submapSizes[bottomLeft][1]);
  EXPECT_EQ(0, submapSizes[bottomRight][0]);
  EXPECT_EQ(0, submapSizes[bottomRight][1]);

  submapSize << 0, 7;
  EXPECT_FALSE(getBufferRegionsForSubmap(submapIndeces, submapSizes, submapIndex, submapSize, bufferSize));

  submapSize << 6, 7;
  EXPECT_FALSE(getBufferRegionsForSubmap(submapIndeces, submapSizes, submapIndex, submapSize, bufferSize));
}

TEST(getBufferRegionsForSubmap, Simple)
{
  unsigned int topLeft = bufferRegionIndeces[bufferRegion::TopLeft];
  unsigned int topRight = bufferRegionIndeces[bufferRegion::TopRight];
  unsigned int bottomLeft= bufferRegionIndeces[bufferRegion::BottomLeft];
  unsigned int bottomRight= bufferRegionIndeces[bufferRegion::BottomRight];

  Eigen::Array2i submapIndex;
  Eigen::Array2i submapSize;
  Eigen::Array2i bufferSize(5, 4);
  vector<Eigen::Array2i> submapIndeces, submapSizes;

  submapIndex << 1, 2;
  submapSize << 3, 2;
  EXPECT_TRUE(getBufferRegionsForSubmap(submapIndeces, submapSizes, submapIndex, submapSize, bufferSize));
  EXPECT_EQ(1, submapIndeces[topLeft][0]);
  EXPECT_EQ(2, submapIndeces[topLeft][1]);
  EXPECT_EQ(3, submapSizes[topLeft][0]);
  EXPECT_EQ(2, submapSizes[topLeft][1]);
  EXPECT_EQ(0, submapSizes[topRight][0]);
  EXPECT_EQ(0, submapSizes[topRight][1]);
  EXPECT_EQ(0, submapSizes[bottomLeft][0]);
  EXPECT_EQ(0, submapSizes[bottomLeft][1]);
  EXPECT_EQ(0, submapSizes[bottomRight][0]);
  EXPECT_EQ(0, submapSizes[bottomRight][1]);
}

TEST(getBufferRegionsForSubmap, CircularBuffer)
{
  unsigned int topLeft = bufferRegionIndeces[bufferRegion::TopLeft];
  unsigned int topRight = bufferRegionIndeces[bufferRegion::TopRight];
  unsigned int bottomLeft= bufferRegionIndeces[bufferRegion::BottomLeft];
  unsigned int bottomRight= bufferRegionIndeces[bufferRegion::BottomRight];

  Eigen::Array2i submapIndex;
  Eigen::Array2i submapSize;
  Eigen::Array2i bufferSize(5, 4);
  Eigen::Array2i bufferStartIndex(3, 1);
  vector<Eigen::Array2i> submapIndeces, submapSizes;

  submapIndex << 3, 1;
  submapSize << 2, 3;
  EXPECT_TRUE(getBufferRegionsForSubmap(submapIndeces, submapSizes, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, submapIndeces[topLeft][0]);
  EXPECT_EQ(1, submapIndeces[topLeft][1]);
  EXPECT_EQ(2, submapSizes[topLeft][0]);
  EXPECT_EQ(3, submapSizes[topLeft][1]);
  EXPECT_EQ(0, submapSizes[topRight][0]);
  EXPECT_EQ(0, submapSizes[topRight][1]);
  EXPECT_EQ(0, submapSizes[bottomLeft][0]);
  EXPECT_EQ(0, submapSizes[bottomLeft][1]);
  EXPECT_EQ(0, submapSizes[bottomRight][0]);
  EXPECT_EQ(0, submapSizes[bottomRight][1]);

  submapIndex << 4, 1;
  submapSize << 2, 3;
  getBufferRegionsForSubmap(submapIndeces, submapSizes, submapIndex, submapSize, bufferSize, bufferStartIndex);
  EXPECT_EQ(4, submapIndeces[topLeft][0]);
  EXPECT_EQ(1, submapIndeces[topLeft][1]);
  EXPECT_EQ(1, submapSizes[topLeft][0]);
  EXPECT_EQ(3, submapSizes[topLeft][1]);
  EXPECT_EQ(0, submapSizes[topRight][0]);
  EXPECT_EQ(0, submapSizes[topRight][1]);
  EXPECT_EQ(0, submapIndeces[bottomLeft][0]);
  EXPECT_EQ(1, submapIndeces[bottomLeft][1]);
  EXPECT_EQ(1, submapSizes[bottomLeft][0]);
  EXPECT_EQ(3, submapSizes[bottomLeft][1]);
  EXPECT_EQ(0, submapSizes[bottomRight][0]);
  EXPECT_EQ(0, submapSizes[bottomRight][1]);

  submapIndex << 1, 0;
  submapSize << 2, 1;
  EXPECT_TRUE(getBufferRegionsForSubmap(submapIndeces, submapSizes, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(0, submapSizes[topLeft][0]);
  EXPECT_EQ(0, submapSizes[topLeft][1]);
  EXPECT_EQ(0, submapSizes[topRight][0]);
  EXPECT_EQ(0, submapSizes[topRight][1]);
  EXPECT_EQ(0, submapSizes[bottomLeft][0]);
  EXPECT_EQ(0, submapSizes[bottomLeft][1]);
  EXPECT_EQ(1, submapIndeces[bottomRight][0]);
  EXPECT_EQ(0, submapIndeces[bottomRight][1]);
  EXPECT_EQ(2, submapSizes[bottomRight][0]);
  EXPECT_EQ(1, submapSizes[bottomRight][1]);

  submapIndex << 3, 1;
  submapSize << 5, 4;
  EXPECT_TRUE(getBufferRegionsForSubmap(submapIndeces, submapSizes, submapIndex, submapSize, bufferSize, bufferStartIndex));
  EXPECT_EQ(3, submapIndeces[topLeft][0]);
  EXPECT_EQ(1, submapIndeces[topLeft][1]);
  EXPECT_EQ(2, submapSizes[topLeft][0]);
  EXPECT_EQ(3, submapSizes[topLeft][1]);
  EXPECT_EQ(3, submapIndeces[topRight][0]);
  EXPECT_EQ(0, submapIndeces[topRight][1]);
  EXPECT_EQ(2, submapSizes[topRight][0]);
  EXPECT_EQ(1, submapSizes[topRight][1]);
  EXPECT_EQ(0, submapIndeces[bottomLeft][0]);
  EXPECT_EQ(1, submapIndeces[bottomLeft][1]);
  EXPECT_EQ(3, submapSizes[bottomLeft][0]);
  EXPECT_EQ(3, submapSizes[bottomLeft][1]);
  EXPECT_EQ(0, submapIndeces[bottomRight][0]);
  EXPECT_EQ(0, submapIndeces[bottomRight][1]);
  EXPECT_EQ(3, submapSizes[bottomRight][0]);
  EXPECT_EQ(1, submapSizes[bottomRight][1]);
}
