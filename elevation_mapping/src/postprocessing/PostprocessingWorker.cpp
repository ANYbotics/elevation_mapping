/*
 * PostprocessingWorker.cpp
 *
 *  Created on: Dec. 21, 2020
 *      Author: Yoshua Nava
 *   Institute: ANYbotics
 */

#include "elevation_mapping/postprocessing/PostprocessingWorker.hpp"

namespace elevation_mapping {

PostprocessingWorker::PostprocessingWorker(ros::NodeHandle nodeHandle)
    : functor_(nodeHandle), work_(ioService_), thread_([this] { this->ioService_.run(); }) {}

PostprocessingWorker::GridMap PostprocessingWorker::processBuffer() {
  return functor_(dataBuffer_);
}

void PostprocessingWorker::publish(const GridMap& gridMap) const {
  functor_.publish(gridMap);
}

bool PostprocessingWorker::hasSubscribers() const {
  return functor_.hasSubscribers();
}

}  // namespace elevation_mapping
