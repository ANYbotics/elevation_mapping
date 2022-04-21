/*
 * PostprocessingPipelineFunctor.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 *
 *  Note. Large parts are adopted from grid_map_demos/FiltersDemo.cpp.
 */

#include <grid_map_ros/grid_map_ros.hpp>

#include "elevation_mapping/postprocessing/PostprocessingPipelineFunctor.hpp"

namespace elevation_mapping {

PostprocessingPipelineFunctor::PostprocessingPipelineFunctor(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), filterChain_("grid_map::GridMap"), filterChainConfigured_(false) {
  // TODO (magnus) Add logic when setting up failed. What happens actually if it is not configured?
  readParameters();
  const Parameters parameters{parameters_.getData()};
  publisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>(parameters.outputTopic_, 1, true);

  // Setup filter chain.
  if (!nodeHandle.hasParam(parameters.filterChainParametersName_) ||
      !filterChain_.configure(parameters.filterChainParametersName_, nodeHandle)) {
    ROS_WARN("Could not configure the filter chain. Will publish the raw elevation map without postprocessing!");
    return;
  }

  filterChainConfigured_ = true;
}

PostprocessingPipelineFunctor::~PostprocessingPipelineFunctor() = default;

void PostprocessingPipelineFunctor::readParameters() {
  Parameters parameters;
  nodeHandle_.param("output_topic", parameters.outputTopic_, std::string("elevation_map_raw"));
  nodeHandle_.param("postprocessor_pipeline_name", parameters.filterChainParametersName_, std::string("postprocessor_pipeline"));
  parameters_.setData(parameters);
}

grid_map::GridMap PostprocessingPipelineFunctor::operator()(GridMap& inputMap) {
  if (not filterChainConfigured_) {
    ROS_WARN_ONCE("No postprocessing pipeline was configured. Forwarding the raw elevation map!");
    return inputMap;
  }

  grid_map::GridMap outputMap;
  if (not filterChain_.update(inputMap, outputMap)) {
    ROS_ERROR("Could not perform the grid map filter chain! Forwarding the raw elevation map!");
    return inputMap;
  }

  return outputMap;
}

void PostprocessingPipelineFunctor::publish(const GridMap& gridMap) const {
  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  grid_map::GridMapRosConverter::toMessage(gridMap, outputMessage);
  publisher_.publish(outputMessage);
  ROS_DEBUG("Elevation map raw has been published.");
}

bool PostprocessingPipelineFunctor::hasSubscribers() const {
  return publisher_.getNumSubscribers() > 0;
}

}  // namespace elevation_mapping
