/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include <ros/ros.h>
#include "elevation_mapping/ElevationMapping.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_mapping");
  ros::NodeHandle nodeHandle("~");
  elevation_mapping::ElevationMapping elevationMap(nodeHandle);

  // Spin
  ros::AsyncSpinner spinner(nodeHandle.param("num_callback_threads", 1));  // Use n threads
  spinner.start();
  ros::waitForShutdown();
  return 0;
}
