/*
 * test_elevation_mapping.cpp
 *
 *  Created on: Nov 27, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include <cstdio>

// gtest
#include <gtest/gtest.h>

// Run all the tests that were declared with TEST()
int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  int initValue = static_cast<int>(time(nullptr));
  std::cout << "Init value for random number generator: " << initValue << std::endl;
  srand(initValue);
  return RUN_ALL_TESTS();
}
