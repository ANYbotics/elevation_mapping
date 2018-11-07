/*
 * elevation_layer.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: Eugenio Chisari
 *	 Institute: ANYbotics
 */

#include <elevation_layer/elevation_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(elevation_layer::ElevationLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

using costmap_2d::Observation;
using costmap_2d::ObservationBuffer;

namespace elevation_layer {

ElevationLayer::ElevationLayer() : filterChain_("grid_map::GridMap"), elevation_map_received_(false), filters_configuration_loaded_(false) {
  costmap_ = nullptr;  // this is the unsigned char* member of parent class Costmap2D.
}

void ElevationLayer::onInitialize() {
  ros::NodeHandle nh("~/" + name_);
  rolling_window_ = layered_costmap_->isRolling();

  ElevationLayer::matchSize();
  current_ = true;
  elevation_map_received_ = false;
  filters_configuration_loaded_ = false;
  global_frame_ = layered_costmap_->getGlobalFrameID();

  // get our tf prefix
  ros::NodeHandle prefix_nh;
  const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

  // get parameters from config file
  if (!nh.param("elevation_topic", elevation_topic_, std::string(""))) {
    ROS_WARN("did not find elevation_topic, using default");
  }
  if (!nh.param("height_threshold", height_threshold_, 0.0)) {
    ROS_WARN("did not find height_treshold, using default");
  }
  if (!nh.param("filter_chain_parameters_name", filter_chain_parameters_name_, std::string(""))) {
    ROS_WARN("did not find filter_chain_param_name, using default");
  }
  if (!nh.param("elevation_layer_name", elevation_layer_name_, std::string(""))) {
    ROS_WARN("did not find elevation_layer_name, using default");
  }
  if (!nh.param("edges_layer_name", edges_layer_name_, std::string(""))) {
    ROS_WARN("did not find edges_layer_name, using default");
  }
  if (!nh.param("footprint_clearing_enabled", footprint_clearing_enabled_, false)) {
    ROS_WARN("did not find footprint_clearing_enabled, using default");
  }
  if (!nh.param("edges_sharpness_threshold", edges_sharpness_threshold_, 0.0)) {
    ROS_WARN("did not find edges_sharpness_treshold, using default");
  }
  bool track_unknown_space = layered_costmap_->isTrackingUnknown();
  if (!nh.param("track_unknown_space", track_unknown_space, false)) {
    ROS_WARN("did not find track_unknown_space, using default");
  }
  default_value_ = track_unknown_space ? NO_INFORMATION : FREE_SPACE;
  std::string combination_method;
  if (!nh.param("combination_method", combination_method, std::string(""))) {
    ROS_WARN("did not find combination_method, using default");
  }
  combination_method_ = convertCombinationMethod(combination_method);

  // Subscribe to topic
  elevation_subscriber_ = nh.subscribe(elevation_topic_, 1, &ElevationLayer::elevationMapCallback, this);
  dsrv_ = nullptr;
  setupDynamicReconfigure(nh);

  // Setup filter chain.
  if (!filterChain_.configure(filter_chain_parameters_name_, nh)) {
    ROS_WARN("Could not configure the filter chain!");
  } else {
    filters_configuration_loaded_ = true;
  }
}

void ElevationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                  double* max_y) {
  std::lock_guard<std::mutex> lock(elevation_map_mutex_);
  if (rolling_window_) updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  if (!(enabled_ && elevation_map_received_)) return;
  useExtraBounds(min_x, min_y, max_x, max_y);

  for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridmap_index(*iterator);
    grid_map::Position vertexPositionXY;
    elevation_map_.getPosition(gridmap_index, vertexPositionXY);
    double px = vertexPositionXY.x();
    double py = vertexPositionXY.y();

    touch(px, py, min_x, min_y, max_x, max_y);
  }
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ElevationLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                     double* max_y) {
  if (!footprint_clearing_enabled_) return;
  costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

  for (auto& i : transformed_footprint_) {
    touch(i.x, i.y, min_x, min_y, max_x, max_y);
  }
}

void ElevationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  std::lock_guard<std::mutex> lock(elevation_map_mutex_);
  if (!enabled_ || !elevation_map_received_) return;
  const bool has_edges_layer = elevation_map_.exists(edges_layer_name_);
  if (!has_edges_layer) {
    ROS_WARN_THROTTLE(0.2, "No edges layer found !!");
  }
  ros::Duration time_since_elevation_map_received = ros::Time::now() - last_elevation_map_update_;
  if (time_since_elevation_map_received > ros::Duration(5.0)) {
    current_ = false;
  }
  const grid_map::Matrix& elevation_data = elevation_map_[elevation_layer_name_];
  for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridmap_index(*iterator);
    grid_map::Position vertexPositionXY;
    elevation_map_.getPosition(gridmap_index, vertexPositionXY);
    double px = vertexPositionXY.x();
    double py = vertexPositionXY.y();
    // now we need to compute the map coordinates for the observation
    unsigned int mx, my;
    if (!worldToMap(px, py, mx, my))  // if point outside of local costmap, ignore
    {
      continue;
    }
    if (elevation_data(gridmap_index(0), gridmap_index(1)) > height_threshold_)  // If point too high, it could be an obstacle
    {
      if (has_edges_layer) {
        const grid_map::Matrix& edges_data = elevation_map_[edges_layer_name_];
        if (edges_data(gridmap_index(0), gridmap_index(1)) < edges_sharpness_threshold_)  // if area not sharp, dont label as obstacle
        {
          setCost(mx, my, FREE_SPACE);
          continue;
        }
      }
      setCost(mx, my, LETHAL_OBSTACLE);
    } else {
      setCost(mx, my, FREE_SPACE);
    }
  }

  if (footprint_clearing_enabled_) {
    setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
  }

  switch (combination_method_) {
    case Overwrite:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case Maximum:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Do Nothing
      break;
  }
}

CombinationMethod convertCombinationMethod(const std::string& str) {
  if (str == "Overwrite") {
    return Overwrite;
  } else if (str == "Maximum") {
    return Maximum;
  } else {
    ROS_WARN_THROTTLE(0.2, "Unknow combination method !");
    return Unknown;
  }
}

void ElevationLayer::elevationMapCallback(const grid_map_msgs::GridMapConstPtr& elevation) {
  grid_map::GridMap incoming_map;
  grid_map::GridMap filtered_map;
  if (!grid_map::GridMapRosConverter::fromMessage(*elevation, incoming_map)) {
    ROS_WARN_THROTTLE(0.2, "Grid Map msg Conversion failed !");
    return;
  }
  last_elevation_map_update_ = ros::Time::now();
  incoming_map.convertToDefaultStartIndex();
  if (!(global_frame_ == incoming_map.getFrameId())) {
    ROS_WARN_THROTTLE(0.2, "Incoming elevation_map frame different than expected! ");
  }
  // Apply filter chain.
  if (filters_configuration_loaded_ && filterChain_.update(incoming_map, filtered_map)) {
    std::lock_guard<std::mutex> lock(elevation_map_mutex_);
    elevation_map_ = filtered_map;
    height_threshold_ /= 2.0;  // Half the threshold since the highest sharpness is at midheigth of the obstacles
  } else {
    std::lock_guard<std::mutex> lock(elevation_map_mutex_);
    ROS_WARN_THROTTLE(0.2, "Could not use the filter chain!");
    elevation_map_ = incoming_map;
  }
  if (!elevation_map_received_) {
    //            elevation_map_received_.store(true);
    elevation_map_received_ = true;
  }
}

void ElevationLayer::setupDynamicReconfigure(ros::NodeHandle& nh) {
  dsrv_.reset(new dynamic_reconfigure::Server<elevation_layer::ElevationPluginConfig>(nh));
  dynamic_reconfigure::Server<elevation_layer::ElevationPluginConfig>::CallbackType cb =
      boost::bind(&ElevationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void ElevationLayer::reconfigureCB(elevation_layer::ElevationPluginConfig& config, uint32_t level) { enabled_ = config.enabled; }

void ElevationLayer::reset() {
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

void ElevationLayer::activate() {
  // if we're stopped we need to re-subscribe to topics
  ros::NodeHandle nh("~/" + name_);
  elevation_subscriber_ = nh.subscribe(elevation_topic_, 1, &ElevationLayer::elevationMapCallback, this);
}
void ElevationLayer::deactivate() { elevation_subscriber_.shutdown(); }
}  // namespace elevation_layer