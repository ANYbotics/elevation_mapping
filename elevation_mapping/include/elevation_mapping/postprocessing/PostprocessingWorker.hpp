/*
 * PostprocessingWorker.hpp
 *
 *  Created on: Dec. 21, 2020
 *      Author: Yoshua Nava
 *   Institute: ANYbotics
 */

#pragma once

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <thread>

#include <grid_map_core/GridMap.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace elevation_mapping {

/**
 * @brief A wrapper around the postprocessing pipelines functor.
 *
 * @remark It stores together the functor, grid map data buffer, and thread tooling.
 * It is assumed that the members of this function are guarded by an external mutex,
 * handled by the owner of this class.
 */
class PostprocessingWorker {
 public:
  using GridMap = grid_map::GridMap;

  explicit PostprocessingWorker(ros::NodeHandle nodeHandle);

  /*! @name Accessors */
  ///@{
  boost::asio::io_service& ioService() { return ioService_; }
  std::thread& thread() { return thread_; }
  const GridMap& dataBuffer() { return dataBuffer_; }
  void setDataBuffer(GridMap data) { dataBuffer_ = std::move(data); }
  ///@}

  /*! @name Methods */
  ///@{
  /**
   * @brief Process the data in the buffer.
   *
   * @return GridMap Processed grid map.
   */
  GridMap processBuffer();

  /**
   * @brief Publish a given grid map.
   *
   * @param gridMap The grid map to publish.
   */
  void publish(const GridMap& gridMap) const;

  /**
   * @brief Checks whether the worker publisher has any active subscribers.
   *
   * @return true If there are subscribers to the worker publisher, false otherwise.
   */
  bool hasSubscribers() const;
  ///@}

 protected:
  //! The functor to execute on a given GridMap.
  PostprocessingPipelineFunctor functor_;

  //! BOOST Service Worker Infrastructure.
  //! The io_service objects provide the interface to post an asynchronous task.
  //! The work object ensures that the io_service run() method keeps spinning and accepts tasks.
  //! A thread executes the io_service::run() method, which does io_service::work, ie accepting and executing new tasks.
  //! IO service for asynchronous operation.
  boost::asio::io_service ioService_;
  //! IO service work notifier
  boost::asio::io_service::work work_;
  //! The thread on which this worker runs.
  std::thread thread_;

  //! Data container for the worker.
  GridMap dataBuffer_;
};

}  // namespace elevation_mapping
