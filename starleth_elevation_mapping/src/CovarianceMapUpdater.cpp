/*
 * CovarianceMapUpdater.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */
#include "CovarianceMapUpdater.hpp"

namespace starleth_elevation_mapping {

CovarianceMapUpdater::CovarianceMapUpdater()
{
  // TODO Auto-generated constructor stub
  robotPoseCovariance_.setZero();
}

CovarianceMapUpdater::~CovarianceMapUpdater()
{
  // TODO Auto-generated destructor stub
}

} /* namespace starleth_elevation_mapping */
