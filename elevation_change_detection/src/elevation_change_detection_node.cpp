/*
 * elevation_change_detection_node.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include <ros/ros.h>

#include "elevation_change_detection/ElevationChangeDetection.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_change_detection");
  ros::NodeHandle nodeHandle("~");
//  elevation_change_detection::ElevationChangeDetection ElevationChangeDetection(nodeHandle);

  // Spin
  ros::spin();
  return 0;
}
