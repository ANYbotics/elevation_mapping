/*
 * ElevationVisualization.hpp
 *
 *  Created on: Nov 19, 2013
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 *
 */

#pragma once

// ROS
#include <ros/ros.h>

namespace starleth {

/*
 *
 */
class ElevationVisualization
{
 public:
  ElevationVisualization(ros::NodeHandle& nodeHandle);

  virtual ~ElevationVisualization();

  bool update();

 private:

};

} /* namespace footholdfinder */
