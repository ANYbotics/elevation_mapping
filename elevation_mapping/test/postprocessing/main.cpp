#include <gtest/gtest.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_mapping");
  ros::start();  // To make use of ROS time in output macros.
  testing::InitGoogleTest(&argc, argv);
  int res = RUN_ALL_TESTS();
  ros::shutdown();
  return res;
}
