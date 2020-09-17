/*
 * PostprocessingPipelineFunctor.cpp
 *
 *  Created on: Sep. 14, 2020
 *      Author: Magnus Gärtner
 *   Institute: ETH Zurich, ANYbotics
 */

#pragma once

#include <filters/filter_chain.h>
#include <ros/ros.h>
#include <grid_map_core/GridMap.hpp>

namespace elevation_mapping {

/**
 * @brief A configurable postprocessing functor, it applies the configured filter pipeline to the input.
 *
 *   Usage:
 *   ========
 *
 *   // Create the functor, it will configure the postprocessing pipeline from the ros parameters.
 *   PostprocessingPipelineFunctor postprocessor(nodeHandle);
 *
 *   // Call the functor by feeding it some input data. It will postprocess and publish the processed data.
 *   postprocessor(gridMap);
 *
 */
class PostprocessingPipelineFunctor {
 public:
  using GridMap = grid_map::GridMap;

  /**
   * @brief Constructor.
   * @param nodeHandle The node handle to read parameters from and to publish output data.
   */
  explicit PostprocessingPipelineFunctor(ros::NodeHandle& nodeHandle);

  /**
   * @brief Destructor.
   */
  ~PostprocessingPipelineFunctor();

  /**
   * @brief The operator. It applies this object, e.g a postprocessing pipeline to the given GridMap.
   * @param inputMap The gridMap on which the postprocessing is applied (not inplace).
   * @return The postprocessed gridMap.
   */
  GridMap operator()(GridMap& inputMap);

  /**
   * Publishes a given grid map.
   * @param gridMap   The Grid Map that this functor will publish.
   */
  void publish(GridMap& gridMap) const;

  /**
   * Checks whether there are any subscribers to the result of this pipeline.
   *
   * @return True if someone listens to the topic this task publishes to.
   */
  bool pipelineHasSubscribers() const;

 private:
  /**
   * @brief Reads in the parameters from the ROS parameter server.
   * @return A flag whether the parameters were read successfully.
   */
  void readParameters();

  //! ROS nodehandle.
  ros::NodeHandle& nodeHandle_;

  //! Name of the output grid map topic.
  std::string outputTopic_;

  //! Grid map publisher.
  ros::Publisher publisher_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name.
  std::string filterChainParametersName_;

  //! Flag indicating if the filter chain was successfully configured.
  bool filterChainConfigured_;
};

}  // namespace elevation_mapping