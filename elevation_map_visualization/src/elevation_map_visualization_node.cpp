/*
 * elevation_map_visualization_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "elevation_map_visualization/ElevationMapVisualization.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_map_visualization");

  // Logger
#if ROS_VERSION_MINIMUM(1, 10, 0) // Hydro and newer
  ros::console::set_logger_level("Info", ros::console::levels::Info);
  ros::console::notifyLoggerLevelsChanged();
#else
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
#endif

  ros::NodeHandle nodeHandle("~");

  elevation_map_visualization::ElevationMapVisualization elevationMapVisualization(nodeHandle);

  ros::spin();
  return 0;
}
