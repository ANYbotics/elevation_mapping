/*
 * ElevationMapFunctors.hpp
 *
 *  Created on: Jan 4, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, ANYbotics
 */

#pragma once

namespace elevation_mapping {

template<typename Scalar>
struct VarianceClampOperator
{
  VarianceClampOperator(const Scalar& minVariance, const Scalar& maxVariance)
      : minVariance_(minVariance),
        maxVariance_(maxVariance)
  {
  }
  const Scalar operator()(const Scalar& x) const
  {
    return x < minVariance_ ? minVariance_ : (x > maxVariance_ ? std::numeric_limits<float>::infinity() : x);
  }
  Scalar minVariance_, maxVariance_;
};

} /* namespace */
