/*
 * PostprocessorPool.hpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <mutex>
#include <thread>

#include <grid_map_core/GridMap.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace elevation_mapping {

/**
 * @brief A handler for executing postprocessing pipelines in parallel.
 *
 * @remark This class starts poolSize threads.
 * It is intentional that if raw maps come in faster than the post processing works some raw maps are skipped.
 * There is a minimal critical section updating the availableServices_ structure.
 * Workers operate on copies of the raw elevation map instead of working on the same data.
 */
class PostprocessorPool {
 public:
  using GridMap = grid_map::GridMap;

  /**
   * @brief Constructor.
   * @param poolSize The number of worker threads to allocate.
   * @param nodeHandle The node handle used to configure the task to run and to publish the finished tasks.
   */
  PostprocessorPool(std::size_t poolSize, ros::NodeHandle nodeHandle);

  //! @brief Destructor.
  ~PostprocessorPool();

  /**
   * @brief Starts a task on a thread from the thread pool if there are some available.
   * @param gridMap The data to be processed by this task.
   * @return True if the PostprocessorPool accepted the task. If false the PostprocessorPool had no available threads and discarded the
   * task.
   */
  bool runTask(const GridMap& gridMap);

  /**
   * @brief Performs a check on the number of subscribers.
   * @return True if someone listens to the topic that the managed postprocessor_ publishes to.
   */
  bool pipelineHasSubscribers() const;

 private:
  /**
   * @brief Wrap a task so that the postprocessor pool gets notified on completion of the task.
   * @param serviceIndex The index of the thread / service that will process this task.
   */
  void wrapTask(size_t serviceIndex);

  //! The functor to execute on a given GridMap.
  PostprocessingPipelineFunctor postprocessor_;

  //! Multiple service workers, all addressed with their corresponding index.
  //! The io_service objects provide the interface to post an asynchronous task.
  //! The work object ensures that the io_service run() method keeps spinning and accepts tasks.
  //! A thread executes the io_service::run() method, which does io_service::work, ie accepting and executing new tasks.

  //! Service objects, object i runs work_[i] on threads_[i].
  std::vector<boost::asio::io_service> ioService_;
  //! The routines to accept is_service tasks.
  std::vector<boost::asio::io_service::work> work_;
  //! Data container for the workers. E.g ioService_[i] operates on dataBuffers_[i].
  std::vector<GridMap> dataBuffers_;
  //! The threads on which the services run. E.g ioService_[i] runs on threads_[i].
  std::vector<std::thread> threads_;

  //! Container holding the service ids which have corresponding threads. The only object that is used in a mutual exclusive manner and must
  //! be protected by availableServicesMutex_.
  std::deque<size_t> availableServices_;
  boost::mutex availableServicesMutex_;
};

}  // namespace elevation_mapping
