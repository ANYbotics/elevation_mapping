/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_mapping");

  // Logger
  ros::console::set_logger_level("Debug", ros::console::levels::Debug);
  ros::console::notifyLoggerLevelsChanged();

  ros::NodeHandle nodeHandle("~");

  elevation_mapping::ElevationMapping elevationMap(nodeHandle);

  // Spin
  ros::AsyncSpinner spinner(1);  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
