/*
 * elevation_layer.h
 *
 *  Created on: Nov 5, 2018
 *      Author: Eugenio Chisari
 *	 Institute: ANYbotics
 */

#pragma once

// C++ Libraries.
#include <atomic>
#include <boost/algorithm/string/predicate.hpp>
#include <mutex>

// Ros headers.
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

// Costmap2d headers.
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/footprint.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>

// Package related headers.
#include <filters/filter_chain.h>
#include <message_filters/subscriber.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include "elevation_mapping_costmap_2d_plugin/ElevationPluginConfig.h"

namespace elevation_mapping_costmap_2d_plugin {

/*!
 * Method to update the cost of a portion of map.
 */
enum CombinationMethod { Overwrite, Maximum, Unknown };

/*!
 * Converts the string from the yaml file to the proper CombinationMethod enum.
 * @param str Input string from yaml file.
 * @return CombinationMethod Enum equivalent.
 */
CombinationMethod convertCombinationMethod(const std::string& str);

/*!
 * Plug-in layer of a costmap_2d derived from elevation_map information.
 */
class ElevationLayer : public costmap_2d::CostmapLayer {
 public:
  /*!
   * Constructor.
   */
  ElevationLayer();

  /*!
   * Destructor.
   */
  ~ElevationLayer() override = default;

  /*!
   * Function called from parent class at initialization.
   */
  void onInitialize() override;

  /*!
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   *
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;

  /*!
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  /*! 
   * @brief Restart publishers if they've been stopped. 
   */
  void activate() override;

  /*!
   * @brief Stop publishers. 
   */
  void deactivate() override;

  /*! 
   * @brief Deactivate, reset the map and then reactivate
   */
  void reset() override;

  /*!
   * Callback to receive the grid_map msg from elevation_map.
   * @param occupancy_grid GridMap msg from elevation_map.
   */
  void elevationMapCallback(const grid_map_msgs::GridMapConstPtr& occupancy_grid);

 protected:
  /*!
   * Set up the dynamic reconfigure.
   * @param nh
   */
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  /*!
   * Clear obstacles inside the footprint of the robort if the flag footprintClearingEnabled_ is true.
   * @param robot_x
   * @param robot_y
   * @param robot_yaw
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);

  //! The global frame for the costmap.
  std::string globalFrame_;

  //! Dynamic reconfigure server.
  std::unique_ptr<dynamic_reconfigure::Server<elevation_mapping_costmap_2d_plugin::ElevationPluginConfig> > dsrv_;

  //! Combination method to use to update the cost of a portion of map.
  CombinationMethod combinationMethod_;

  //! Polygon describing the form of the footprint of the robot.
  std::vector<geometry_msgs::Point> transformedFootprint_;

  //! Whether the local costmap should move together with the robot.
  bool rollingWindow_;

  //! Whether to clean the obstacles inside the robot footprint.
  bool footprintClearingEnabled_;

  //! Whether the elevation_map msg was received.
  std::atomic_bool elevationMapReceived_;

  //! After this time [seconds] without receiving any elevation_map, the robot will have to stop.
  double maxAllowedBlindTime_;

 private:
  /*!
   * Dynamic reconfiguration of the parameters.
   * @param config
   * @param level
   */
  void reconfigureCB(elevation_mapping_costmap_2d_plugin::ElevationPluginConfig& config, uint32_t level);

  ros::NodeHandle nodeHandle_;

  //! The elevation_map from which to take the information abut the environment (filtered or not).
  grid_map::GridMap elevationMap_;

  //! lock_guard mutex to make elevation_map setting thread safe.
  std::mutex elevationMapMutex_;

  //! Ros subscriber to grid_map msgs.
  ros::Subscriber elevationSubscriber_;

  //! Last time an elevation_map was received.
  ros::Time lastElevationMapUpdate_;

  //! Height threshold below which nothing is considered obstacle.
  double heightThreshold_;

  //! Sharpness threshold above which an object is considered an obstacle.
  double edgesSharpnessThreshold_;

  //! Topic_name of the elevation_map incoming msg.
  std::string elevationTopic_;

  //! Filter chain used to filter the incoming elevation_map.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name to use.
  std::string filterChainParametersName_;

  //! Whether filters configuration parameters was found.
  bool filtersConfigurationLoaded_;

  //! Name of the layer of the incoming map to use.
  std::string elevationLayerName_;

  //! Name to give to the filtered layer.
  std::string edgesLayerName_;
};

}  // namespace elevation_mapping_costmap_2d_plugin