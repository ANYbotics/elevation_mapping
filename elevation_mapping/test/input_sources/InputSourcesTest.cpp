/*
 *  InputSourceTest.cpp
 *
 *  Created on: Oct 02, 2020
 *  Author: Magnus Gärtner
 *  Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/ElevationMapping.hpp"

#include <ros/ros.h>

#include <gtest/gtest.h>

static void assertSuccessAndNumberOfSources(const std::string& inputConfiguration, bool successExpected,
                                            uint32_t numberOfExpectedInputSources) {
  elevation_mapping::InputSourceManager inputSourceManager(ros::NodeHandle("~"));
  bool success = inputSourceManager.configureFromRos(inputConfiguration);
  ASSERT_EQ(success, successExpected) << "Configuration was:\n"
                                      << ros::NodeHandle("~").param<XmlRpc::XmlRpcValue>(inputConfiguration, "not set").toXml() << "\n";
  ASSERT_EQ(inputSourceManager.getNumberOfSources(), numberOfExpectedInputSources);
}

TEST(InputSources, SingleInputValid) {  // NOLINT
  assertSuccessAndNumberOfSources("single_valid", true, 1);
}

TEST(InputSources, MultipleInputsValid) {  // NOLINT
  assertSuccessAndNumberOfSources("multiple_valid", true, 3);
}

TEST(InputSources, NoType) {  // NOLINT
  assertSuccessAndNumberOfSources("no_type", false, 0);
}

TEST(InputSources, NoTopic) {  // NOLINT
  assertSuccessAndNumberOfSources("no_topic", false, 0);
}

TEST(InputSources, NoQueueSize) {  // NOLINT
  assertSuccessAndNumberOfSources("no_queue_size", false, 0);
}

TEST(InputSources, NoPublishOnUpdate) {  // NOLINT
  assertSuccessAndNumberOfSources("no_publish_on_update", false, 0);
}

TEST(InputSources, SubscribingSameTwice) {  // NOLINT
  assertSuccessAndNumberOfSources("subscribing_same_topic_twice", false, 1);
}

TEST(InputSources, ConfigurationNotGiven) {  // NOLINT
  assertSuccessAndNumberOfSources("unset_namespace", false, 0);
}

TEST(InputSources, ConfigurationEmptySources) {  // NOLINT
  assertSuccessAndNumberOfSources("empty_sources_list", true, 0);
}

TEST(InputSources, ConfigurationWrongType) {  // NOLINT
  assertSuccessAndNumberOfSources("wrong_type_configuration", false, 0);
}

TEST(InputSources, ConfigurationNotAStruct) {  // NOLINT
  assertSuccessAndNumberOfSources("not_a_struct", false, 0);
}

TEST(InputSources, ConfigurationQueueSizeIsString) {  // NOLINT
  assertSuccessAndNumberOfSources("queue_size_is_string", false, 0);
}

TEST(InputSources, ConfigurationQueueSizeIsNegative) {  // NOLINT
  assertSuccessAndNumberOfSources("negative_queue_size", false, 0);
}

TEST(InputSources, UnknownType) {  // NOLINT
  ros::NodeHandle nodeHandle("~");
  elevation_mapping::InputSourceManager inputSourceManager(nodeHandle);
  inputSourceManager.configureFromRos("unknown_type");

  elevation_mapping::ElevationMapping map{nodeHandle};

  // Trying to register this misconfigured InputSourceManager to our map should fail.
  bool success =
      inputSourceManager.registerCallbacks(map, make_pair("pointcloud", &elevation_mapping::ElevationMapping::pointCloudCallback));
  ASSERT_FALSE(success);
}

TEST(ElevationMap, Constructor) {  // NOLINT
  ros::NodeHandle nodeHandle("~");
  elevation_mapping::ElevationMapping map(nodeHandle);
}

TEST(InputSources, ListeningToTopicsAfterRegistration) {  // NOLINT
  // subscribe to the default parameter "input_sources"
  ros::NodeHandle nodeHandle("~");
  class ElevationMappingWithInputSourcesAccessor : public elevation_mapping::ElevationMapping {
   public:
    explicit ElevationMappingWithInputSourcesAccessor(ros::NodeHandle nodeHandle) : elevation_mapping::ElevationMapping(nodeHandle) {}
    ~ElevationMappingWithInputSourcesAccessor() override = default;
    int getNumberOfSources() { return inputSources_.getNumberOfSources(); }
  } map{nodeHandle};

  // Wait a bit.
  ros::spinOnce();
  ros::Duration(1.0).sleep();
  ros::spinOnce();

  // Publish to the topics we expect map to subscribe.
  ros::NodeHandle nh("");
  ros::Publisher firstLidarPublisher = nh.advertise<sensor_msgs::PointCloud2>("/lidar_1/depth/points", 1, false);
  ros::Publisher secondLidarPublisher = nh.advertise<sensor_msgs::PointCloud2>("/lidar_2/depth/points", 1, false);

  // Check if we have exactly one subscriber per topic.
  ASSERT_EQ(firstLidarPublisher.getNumSubscribers(), 1);
  ASSERT_EQ(secondLidarPublisher.getNumSubscribers(), 1);
  // ASSERT_EQ(firstDepthImagePublisher.getNumSubscribers(), 1);
  ASSERT_EQ(map.getNumberOfSources(), 2);
}
