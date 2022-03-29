/*
 * WeightedEmpiricalCumulativeDistributionFunction.hpp
 *
 *  Created on: Dec 7, 2015
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <iostream>
#include <map>
#include <stdexcept>

namespace elevation_mapping {

template <typename Type>
class WeightedEmpiricalCumulativeDistributionFunction {
 public:
  WeightedEmpiricalCumulativeDistributionFunction() : totalWeight_(0.0), isComputed_(false) {}

  virtual ~WeightedEmpiricalCumulativeDistributionFunction() = default;

  void add(const Type value, const double weight = 1.0) {
    isComputed_ = false;
    if (data_.find(value) != data_.end()) {
      data_[value] += weight;
    } else {
      data_.insert(std::pair<Type, double>(value, weight));
    }
    totalWeight_ += weight;
  }

  void clear() {
    isComputed_ = false;
    totalWeight_ = 0.0;
    data_.clear();
    distribution_.clear();
  }

  bool compute() {
    if (data_.empty()) {
      return false;
    }
    distribution_.clear();
    inverseDistribution_.clear();

    if (data_.size() == 1) {
      // Special treatment for size 1.
      inverseDistribution_.insert(std::pair<double, Type>(0.0, data_.begin()->first));
      inverseDistribution_.insert(std::pair<double, Type>(1.0, data_.begin()->first));
      return isComputed_ = true;
    }

    //    double cumulativeWeight = 0.0;
    //    inverseDistribution_.insert(std::pair<double, Type>(0.0, data_.begin()->first));
    //    for (const auto& point : data_) {
    //      cumulativeWeight += point.second;
    //      inverseDistribution_.insert(inverseDistribution_.end(),
    //                           std::pair<double, Type>(cumulativeWeight / totalWeight_, point.first));
    //    }

    double cumulativeWeight = -data_.begin()->second;  // Smallest observation corresponds to a probability of 0.
    const double adaptedTotalWeight = totalWeight_ - data_.begin()->second;
    for (const auto& point : data_) {
      cumulativeWeight += point.second;
      inverseDistribution_.insert(inverseDistribution_.end(), std::pair<double, Type>(cumulativeWeight / adaptedTotalWeight, point.first));
    }

    return isComputed_ = true;
  }

  /*!
   * Returns the quantile corresponding to the given probability (inverse distribution function).
   * The smallest observation corresponds to a probability of 0 and the largest to a probability of 1.
   * Uses linear interpolation, see https://stat.ethz.ch/R-manual/R-devel/library/stats/html/quantile.html
   * and "Sampling Quantiles in Statistical Packages", Hyndman et. al., 1996.
   * @param probability the order of the quantile.
   * @return the quantile for the given probability.
   */
  Type quantile(const double probability) const {
    if (!isComputed_) {
      throw std::runtime_error(
          "WeightedEmpiricalCumulativeDistributionFunction::quantile(...): The distribution functions needs to be computed (compute()) "
          "first.");
    }
    if (probability <= 0.0) {
      return inverseDistribution_.begin()->second;
    }
    if (probability >= 1.0) {
      return inverseDistribution_.rbegin()->second;
    }
    const auto& up = inverseDistribution_.lower_bound(probability);  // First element that is not less than key.
    auto low = up;                                                   // Copy.
    --low;
    return low->second + (probability - low->first) * (up->second - low->second) / (up->first - low->first);
  }

  friend std::ostream& operator<<(std::ostream& out, const WeightedEmpiricalCumulativeDistributionFunction& wecdf) {
    unsigned int i = 0;
    out << "Data points:" << std::endl;
    for (const auto& point : wecdf.data_) {
      out << "[" << i << "] Value: " << point.first << " Weight: " << point.second << std::endl;
      ++i;
    }

    i = 0;
    out << "Cumulative distribution function:" << std::endl;
    for (const auto& point : wecdf.distribution_) {
      out << "[" << i << "] Value: " << point.first << " Prob.: " << point.second << std::endl;
      ++i;
    }

    i = 0;
    out << "Inverse distribution function:" << std::endl;
    for (const auto& point : wecdf.inverseDistribution_) {
      out << "[" << i << "] Prob.: " << point.first << " Value: " << point.second << std::endl;
      ++i;
    }

    return out;
  }

 private:
  //! Data points stored as value/weight pair (histogram).
  std::map<Type, double> data_;

  //! Cumulative distribution function (and its inverse) stored as value pair/cumulative probability.
  std::map<Type, double> distribution_;
  std::map<double, Type> inverseDistribution_;

  //! Total weight.
  double totalWeight_;

  //! True of computed.
  bool isComputed_;
};

} /* namespace elevation_mapping */
