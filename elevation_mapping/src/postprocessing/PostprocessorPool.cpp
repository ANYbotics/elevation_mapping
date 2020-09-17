/*
 * PostprocessorPool.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

namespace elevation_mapping {

PostprocessorPool::PostprocessorPool(std::size_t poolSize, ros::NodeHandle nodeHandle)
    : postprocessor_(nodeHandle), ioService_(poolSize), dataBuffers_(poolSize) {
  for (std::size_t i = 0; i < poolSize; ++i) {
    // Create one service per thread
    availableServices_.push_back(i);
    // Generate services and link work to it. Needed to accept tasks and keep the service thread running.
    work_.emplace_back(ioService_.at(i));
    threads_.emplace_back(boost::bind(&boost::asio::io_service::run, &ioService_.at(i)));
  }
}

PostprocessorPool::~PostprocessorPool() {
  // Force all threads to return from io_service::run().
  for (auto& service : ioService_) {
    service.stop();
  }

  // Suppress all exceptions. Try to join every worker thread.
  for (auto& thread : threads_) {
    try {
      if (thread.joinable()) {
        thread.join();
      }
    } catch (const std::exception&) {
    }
  }
}

bool PostprocessorPool::runTask(const GridMap& gridMap) {
  // Get an available service id from the shared services pool in a mutually exclusive manner.
  size_t serviceIndex;
  {
    boost::lock_guard<boost::mutex> lock(availableServicesMutex_);
    if (availableServices_.empty()) {
      return false;
    }
    serviceIndex = availableServices_.back();
    availableServices_.pop_back();
  }

  // Copy data to the buffer for the worker thread.
  dataBuffers_.at(serviceIndex) = gridMap;

  // Create a task with the post-processor and dispatch it.
  auto task = std::bind(&PostprocessorPool::wrapTask, this, serviceIndex);
  ioService_.at(serviceIndex).post(task);
  return true;
}

void PostprocessorPool::wrapTask(size_t serviceIndex) {
  // Run the user supplied task.
  try {
    GridMap postprocessedMap = postprocessor_(dataBuffers_.at(serviceIndex));
    postprocessor_.publish(postprocessedMap);
  }
  // Suppress all exceptions.
  catch (const std::exception& exception) {
    ROS_ERROR_STREAM("Postprocessor pipeline, thread " << serviceIndex << " experienced an error: " << exception.what());
  }

  // Task has finished, so increment count of available threads.
  boost::unique_lock<boost::mutex> lock(availableServicesMutex_);
  availableServices_.push_back(serviceIndex);
}

bool PostprocessorPool::pipelineHasSubscribers() const {
  return postprocessor_.pipelineHasSubscribers();
}

}  // namespace elevation_mapping