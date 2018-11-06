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
#include <param_io/get_param.hpp>

#include <costmap_2d/footprint.h>
#include <dynamic_reconfigure/server.h>
#include <elevation_layer/ElevationPluginConfig.h>
#include <nav_msgs/OccupancyGrid.h>

namespace elevation_layer {
class ElevationLayer : public costmap_2d::CostmapLayer {
 public:
  ElevationLayer();
  ~ElevationLayer() override = default;
  void onInitialize() override;
  void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y) override;
  void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) override;

  void activate() override;
  void deactivate() override;
  void reset() override;

  void elevationMapCallback(const grid_map_msgs::GridMapConstPtr& occupancy_grid);

 protected:
  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::vector<boost::shared_ptr<message_filters::SubscriberBase> >
      elevation_subscribers_;  ///< @brief Used for the observation message filters
  std::unique_ptr< dynamic_reconfigure::Server<elevation_layer::ElevationPluginConfig> > dsrv_;
  virtual void setupDynamicReconfigure(ros::NodeHandle& nh);

  int combination_method_;
  std::vector<geometry_msgs::Point> transformed_footprint_;
  bool rolling_window_;
  bool footprint_clearing_enabled_;
  std::atomic_bool elevation_map_received_;
  void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);

 private:
  void reconfigureCB(elevation_layer::ElevationPluginConfig& config, uint32_t level);
  grid_map::GridMap elevation_map_;
  std::mutex elevation_map_mutex_;
  ros::Subscriber elevation_subscriber_;
  double height_threshold_;
  double edges_sharpness_threshold_;
  std::string elevation_topic_;

  //! Filter chain.
  filters::FilterChain<grid_map::GridMap> filterChain_;

  //! Filter chain parameters name.
  std::string filter_chain_parameters_name_;

  bool filters_configuration_loaded_;
  std::string elevation_layer_name_;
  std::string edges_layer_name_;
};

}  // namespace elevation_layer