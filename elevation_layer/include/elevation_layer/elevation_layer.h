/*
 * elevation_layer.h
 *
 *  Created on: Nov 5, 2018
 *      Author: Eugenio Chisari
 *	 Institute: ANYbotics
 */

#pragma once

#include <atomic>
#include <mutex>

#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <filters/filter_chain.h>
#include <message_filters/subscriber.h>
#include <ros/ros.h>
#include "grid_map_ros/GridMapRosConverter.hpp"

#include <costmap_2d/footprint.h>
#include <dynamic_reconfigure/server.h>
#include <elevation_layer/ElevationPluginConfig.h>
#include <nav_msgs/OccupancyGrid.h>

namespace elevation_layer {

/*!
 * Method to update the cost of a portion of map
 */
enum CombinationMethod { Overwrite, Maximum, Unknown };

/*!
 * converts the string from the yaml file to the proper CombinationMethod enum
 * @param str input string from yaml file
 * @return CombinationMethod enum equivalent
 */
CombinationMethod convertCombinationMethod(const std::string& str);

/*!
 * Plug-in layer of a costmap_2d derived from elevation_map information
 */
class ElevationLayer : public costmap_2d::CostmapLayer {
 public:
  /**
   * Constructor
   */
  ElevationLayer();

  /**
   * Destructor
   */
  ~ElevationLayer() override = default;

  /**
   * Function called from parent class at initialization
   */
  void onInitialize() override;

  /**
   * @brief This is called by the LayeredCostmap to poll this plugin as to how
   *        much of the costmap it needs to update. Each layer can increase
   *        the size of this bounds.
   *
   * For more details, see "Layered Costmaps for Context-Sensitive Navigation",
   * by Lu et. Al, IROS 2014.
   */
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;

  /**
   * @brief Actually update the underlying costmap, only within the bounds
   *        calculated during UpdateBounds().
   */
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  /** @brief Restart publishers if they've been stopped. */
  void activate() override;

  /** @brief Stop publishers. */
  void deactivate() override;

  void reset() override;

  /**
   * Callback to receive the grid_map msg from elevation_map
   * @param occupancy_grid GridMap msg from elevation_map
   */
  void elevationMapCallback(const grid_map_msgs::GridMapConstPtr& occupancy_grid);

 protected:

  /**
   * set up the dynamic reconfigure
   * @param nh
   */
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  /**
   * clear obstacles inside the footprint of the robort if the flag footprint_clearing_enabled_ is true
   * @param robot_x
   * @param robot_y
   * @param robot_yaw
   * @param min_x
   * @param min_y
   * @param max_x
   * @param max_y
   */
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);

  ///< @brief The global frame for the costmap
  std::string global_frame_;

  ///< @brief Used for the observation message filters
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> > elevation_subscribers_;

  ///< @brief dynamic reconfigure server
  std::unique_ptr<dynamic_reconfigure::Server<elevation_layer::ElevationPluginConfig> > dsrv_;

  ///< @brief combination method to use to update the cost of a portion of map
  CombinationMethod combination_method_;

  ///< @brief polygon describing the form of the footprint of the robot
  std::vector<geometry_msgs::Point> transformed_footprint_;

  ///< @brief whether the local costmap should move together with the robot
  bool rolling_window_;

  ///< @brief whether to clean the obstacles inside the robot footprint
  bool footprint_clearing_enabled_;

  ///< @brief whether the elevation_map msg was received
  std::atomic_bool elevation_map_received_;

  ///< @brief after this time [seconds] without receiving any elevation_map, the robot will have to stop
  double max_allowed_blind_time_;

 private:

  /**
   * dynamic reconfiguration of the parameters
   * @param config
   * @param level
   */
  void reconfigureCB(elevation_layer::ElevationPluginConfig& config, uint32_t level);

  ///< @brief the elevation_map from which to take the information abut the environment (filtered or not)
  grid_map::GridMap elevation_map_;

  ///< @brief lock_guard mutex to make elevation_map setting thread safe
  std::mutex elevation_map_mutex_;

  ///< @brief Ros subscriber to grid_map msgs
  ros::Subscriber elevation_subscriber_;

  ///< @brief last time an elevation_map was received
  ros::Time last_elevation_map_update_;

  ///< @brief height threshold below which nothing is considered obstacle
  double height_threshold_;

  ///< @brief sharpness threshold above which an object is considered an obstacle
  double edges_sharpness_threshold_;

  ///< @brief topic_name of the elevation_map incoming msg
  std::string elevation_topic_;

  ///< @brief Filter chain used to filter the incoming elevation_map
  filters::FilterChain<grid_map::GridMap> filterChain_;

  ///< @brief Filter chain parameters name to use
  std::string filter_chain_parameters_name_;

  ///< @brief whether filters configuration parameters was found
  bool filters_configuration_loaded_;

  ///< @brief name of the layer of the incoming map to use
  std::string elevation_layer_name_;

  ///< @brief name to give to the filtered layer
  std::string edges_layer_name_;
};

}  // namespace elevation_layer