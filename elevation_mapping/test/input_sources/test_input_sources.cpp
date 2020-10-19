/*
 * test_elevation_mapping.cpp
 *
 *  Created on: Okt 02, 2020
 *      Author: Magnus Gärtner
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <cstdio>

// gtest
#include <gtest/gtest.h>
#include <ros/ros.h>

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "Tests");
  int initValue = static_cast<int>(time(nullptr));
  ::testing::Test::RecordProperty("Init value for random number generator:", initValue);
  srand(initValue);
  return RUN_ALL_TESTS();
}
