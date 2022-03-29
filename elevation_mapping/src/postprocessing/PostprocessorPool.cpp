/*
 * PostprocessorPool.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/postprocessing/PostprocessorPool.hpp"

namespace elevation_mapping {

PostprocessorPool::PostprocessorPool(std::size_t poolSize, ros::NodeHandle nodeHandle) {
  for (std::size_t i = 0; i < poolSize; ++i) {
    // Add worker to the collection.
    workers_.emplace_back(std::make_unique<PostprocessingWorker>(nodeHandle));
    // Create one service per thread
    availableServices_.push_back(i);
  }
}

PostprocessorPool::~PostprocessorPool() {
  // Force all threads to return from io_service::run().
  for (auto& worker : workers_) {
    worker->ioService().stop();
  }

  // Suppress all exceptions. Try to join every worker thread.
  for (auto& worker : workers_) {
    try {
      if (worker->thread().joinable()) {
        worker->thread().join();
      }
    } catch (const std::exception&) {
    }
  }
}

bool PostprocessorPool::runTask(const GridMap& gridMap) {
  // Get an available service id from the shared services pool in a mutually exclusive manner.
  size_t serviceIndex{0};
  {
    boost::lock_guard<boost::mutex> lock(availableServicesMutex_);
    if (availableServices_.empty()) {
      return false;
    }
    serviceIndex = availableServices_.back();
    availableServices_.pop_back();
  }

  // Copy data to the buffer for the worker.
  workers_.at(serviceIndex)->setDataBuffer(gridMap);

  // Create a task with the post-processor and dispatch it.
  auto task = [this, serviceIndex] { wrapTask(serviceIndex); };
  workers_.at(serviceIndex)->ioService().post(task);
  return true;
}

void PostprocessorPool::wrapTask(size_t serviceIndex) {
  // Run the user supplied task.
  try {
    GridMap postprocessedMap = workers_.at(serviceIndex)->processBuffer();
    workers_.at(serviceIndex)->publish(postprocessedMap);
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
  return std::all_of(workers_.cbegin(), workers_.cend(),
                     [](const std::unique_ptr<PostprocessingWorker>& worker) { return worker->hasSubscribers(); });
}

}  // namespace elevation_mapping