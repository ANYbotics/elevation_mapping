/*!
 * @file    PostprocessorTest.cpp
 * @authors Magnus Gärtner (ANYbotics)
 * @brief   Tests for the PostprocessorPool and the PostprocessingPipeline.
 */

#include <gtest/gtest.h>
#include <ros/ros.h>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"
#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

/**
 * We read in a postprocessing (mock) configuration that takes 150ms to execute. We test whether the postprocessorPool accepts/discards the
 * tasks for various configurations of time between tasks and number of threads in the pool.
 */
class RosFixture : public ::testing::Test {
  void SetUp() override {
    const std::map<std::string, std::string> remappings{};
    ros::init(remappings, "post_processor_ros_test");
    ros::start();
  }

  void TearDown() override { ros::shutdown(); }

 public:
  static void checkAcceptedTasks(uint poolSize, uint timeBetweenConsecutiveTasks, std::vector<bool> expectedAcceptanceOutcomes) {
    // Set up ROS node handle.
    ros::NodeHandle nodeHandle("~");

    elevation_mapping::PostprocessorPool pool{poolSize, nodeHandle};
    int taskNumber = 0;
    for (auto expectedOutcome : expectedAcceptanceOutcomes) {
      bool accepted = pool.runTask(grid_map::GridMap());
      if (expectedOutcome) {
        ASSERT_TRUE(accepted) << "Postprocessor pool accepted task number: " << taskNumber << " although it should.";
      } else {
        ASSERT_FALSE(accepted) << "Postprocessor pool accepted task number: " << taskNumber << " although it should not. ";
      }
      taskNumber++;
      std::this_thread::sleep_for(std::chrono::milliseconds(timeBetweenConsecutiveTasks));
    }
  }
};

TEST_F(RosFixture, FiveTasksOneThreadSimultaneously) {  // NOLINT
  checkAcceptedTasks(1, 0, {true, false, false, false, false});
}

TEST_F(RosFixture, FiveTasksTwoThreadSimultaneously) {  // NOLINT
  checkAcceptedTasks(2, 0, {true, true, false, false, false});
}

TEST_F(RosFixture, EnoughTimeToProcess) {  // NOLINT
  checkAcceptedTasks(1, 200, {true, true, true, true, true});
}

TEST_F(RosFixture, EnoughTimeToProcessWithTwoThreads) {  // NOLINT
  checkAcceptedTasks(2, 100, {true, true, true, true, true});
}

TEST_F(RosFixture, ProcessEverySecond) {  // NOLINT
  checkAcceptedTasks(1, 100, {true, false, true, false, true});
}

TEST_F(RosFixture, TwoThreadsWithMiss) {  // NOLINT
  checkAcceptedTasks(2, 60, {true, true, false, true, true, false});
}
