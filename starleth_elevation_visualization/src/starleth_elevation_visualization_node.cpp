/*
 * starleth_elevation_visualization_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "ElevationVisualization.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "starleth_elevation_visualization");

  // Logger
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);

  ros::NodeHandle nodeHandle;

  starleth_elevation_visualization::ElevationVisualization elevationVisualization(nodeHandle);

  ros::spin();
  return 0;
}
