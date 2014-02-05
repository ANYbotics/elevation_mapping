/*
 * CovarianceMapUpdater.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Eigen
#include <Eigen/Core>

namespace starleth_elevation_mapping {

/*
 *
 */
class CovarianceMapUpdater
{
 public:
  CovarianceMapUpdater();
  virtual ~CovarianceMapUpdater();

 private:

  Eigen::Matrix<double, 6, 6> robotPoseCovariance_;
};

} /* namespace starleth_elevation_mapping */
