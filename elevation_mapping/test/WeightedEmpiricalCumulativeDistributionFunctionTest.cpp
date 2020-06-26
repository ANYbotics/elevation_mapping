/*
 * WeightedEmpiricalCumulativeDistributionFunctionTest.cpp
 *
 *  Created on: Dec 7, 2015
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/WeightedEmpiricalCumulativeDistributionFunction.hpp"

// gtest
#include <gtest/gtest.h>

TEST(WeightedEmpiricalCumulativeDistributionFunction, Initialization) {  // NOLINT
  elevation_mapping::WeightedEmpiricalCumulativeDistributionFunction<float> wecdf;
  EXPECT_FALSE(wecdf.compute());
  wecdf.clear();
  EXPECT_FALSE(wecdf.compute());
}

TEST(WeightedEmpiricalCumulativeDistributionFunction, Trivial) {  // NOLINT
  elevation_mapping::WeightedEmpiricalCumulativeDistributionFunction<double> wecdf;
  wecdf.add(0.0);
  wecdf.add(1.0);
  EXPECT_TRUE(wecdf.compute());
  EXPECT_DOUBLE_EQ(0.0, wecdf.quantile(-0.1));
  EXPECT_DOUBLE_EQ(0.0, wecdf.quantile(0.0));
  EXPECT_DOUBLE_EQ(0.25, wecdf.quantile(0.25));
  EXPECT_DOUBLE_EQ(0.5, wecdf.quantile(0.5));
  EXPECT_DOUBLE_EQ(2.0 / 3.0, wecdf.quantile(2.0 / 3.0));
  EXPECT_DOUBLE_EQ(0.95, wecdf.quantile(0.95));
  EXPECT_DOUBLE_EQ(1.0, wecdf.quantile(1.0));
  EXPECT_DOUBLE_EQ(1.0, wecdf.quantile(1.1));
}

TEST(WeightedEmpiricalCumulativeDistributionFunction, LinearEquallySpaced) {  // NOLINT
  elevation_mapping::WeightedEmpiricalCumulativeDistributionFunction<double> wecdf;
  wecdf.add(0.0);
  wecdf.add(10.0 / 3.0);
  wecdf.add(20.0 / 3.0);
  wecdf.add(10.0);
  EXPECT_TRUE(wecdf.compute());
  EXPECT_DOUBLE_EQ(0.0, wecdf.quantile(0.0));
  EXPECT_DOUBLE_EQ(2.5, wecdf.quantile(0.25));
  EXPECT_DOUBLE_EQ(5.0, wecdf.quantile(0.5));
  EXPECT_DOUBLE_EQ(20.0 / 3.0, wecdf.quantile(2.0 / 3.0));
  EXPECT_DOUBLE_EQ(9.5, wecdf.quantile(0.95));
  EXPECT_DOUBLE_EQ(10.0, wecdf.quantile(1.1));
}

TEST(WeightedEmpiricalCumulativeDistributionFunction, SingleValue) {  // NOLINT
  elevation_mapping::WeightedEmpiricalCumulativeDistributionFunction<double> wecdf;
  wecdf.add(3.0);
  wecdf.add(3.0);
  wecdf.add(3.0);
  EXPECT_TRUE(wecdf.compute());
  EXPECT_DOUBLE_EQ(3.0, wecdf.quantile(0.0));
  EXPECT_DOUBLE_EQ(3.0, wecdf.quantile(0.25));
  EXPECT_DOUBLE_EQ(3.0, wecdf.quantile(0.5));
  EXPECT_DOUBLE_EQ(3.0, wecdf.quantile(1.0));
  EXPECT_DOUBLE_EQ(3.0, wecdf.quantile(2.0));
}

TEST(WeightedEmpiricalCumulativeDistributionFunction, SyntheticDataDebug) {  // NOLINT
  elevation_mapping::WeightedEmpiricalCumulativeDistributionFunction<double> wecdf;
  for (unsigned int i = 0; i < 10; ++i) {
    wecdf.add(1.0);
  }
  wecdf.add(2.0);
  EXPECT_TRUE(wecdf.compute());
  EXPECT_DOUBLE_EQ(1.05, wecdf.quantile(0.05));
  EXPECT_DOUBLE_EQ(1.95, wecdf.quantile(0.95));
}
