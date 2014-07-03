/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "elevation_mapping");

  //Logger
#if ROS_VERSION_MINIMUM(1, 10, 0) // Hydro and newer
  ros::console::set_logger_level("Info", ros::console::levels::Info);
  ros::console::notifyLoggerLevelsChanged();
#else
  log4cxx::LoggerPtr logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  logger->setLevel(ros::console::g_level_lookup[ros::console::levels::Info]);
#endif

  ros::NodeHandle nodeHandle("~");

  std::string sensorTypeName;
  elevation_mapping::SensorType sensorType;
  nodeHandle.param("sensor_type", sensorTypeName, "KINECT");
  if (sensorTypeName == "ASLAM")
    sensorType = elevation_mapping::ASLAM;
  else
    sensorType = elevation_mapping::KINECT;
  elevation_mapping::ElevationMapping elevationMap(nodeHandle, sensorType);

  // Spin
  ros::AsyncSpinner spinner(1); // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
