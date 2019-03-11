/*
 * elevation_layer.cpp
 *
 *  Created on: Nov 5, 2018
 *      Author: Eugenio Chisari
 *	 Institute: ANYbotics
 */

#include "elevation_mapping_costmap_2d_plugin/elevation_layer.hpp"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(elevation_mapping_costmap_2d_plugin::ElevationLayer, costmap_2d::Layer)

using costmap_2d::FREE_SPACE;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

using costmap_2d::Observation;
using costmap_2d::ObservationBuffer;

namespace elevation_mapping_costmap_2d_plugin {

ElevationLayer::ElevationLayer() : filterChain_("grid_map::GridMap"), elevationMapReceived_(false), filtersConfigurationLoaded_(false) {
  costmap_ = nullptr;  // this is the unsigned char* member of parent class Costmap2D.
}

void ElevationLayer::onInitialize() {
  nodeHandle_ = ros::NodeHandle("~/" + name_);
  rollingWindow_ = layered_costmap_->isRolling();

  ElevationLayer::matchSize();
  current_ = true;
  elevationMapReceived_ = false;
  filtersConfigurationLoaded_ = false;
  globalFrame_ = layered_costmap_->getGlobalFrameID();

  // get parameters from config file
  if (!nodeHandle_.param("elevation_topic", elevationTopic_, std::string(""))) {
    ROS_WARN("did not find elevation_topic, using default");
  }
  if (!nodeHandle_.param("height_threshold", heightThreshold_, 0.0)) {
    ROS_WARN("did not find height_treshold, using default");
  }
  if (!nodeHandle_.param("filter_chain_parameters_name", filterChainParametersName_, std::string(""))) {
    ROS_WARN("did not find filter_chain_param_name, using default");
  }
  if (!nodeHandle_.param("elevation_layer_name", elevationLayerName_, std::string(""))) {
    ROS_WARN("did not find elevation_layer_name, using default");
  }
  if (!nodeHandle_.param("edges_layer_name", edgesLayerName_, std::string(""))) {
    ROS_WARN("did not find edges_layer_name, using default");
  }
  if (!nodeHandle_.param("footprint_clearing_enabled", footprintClearingEnabled_, false)) {
    ROS_WARN("did not find footprint_clearing_enabled, using default");
  }
  if (!nodeHandle_.param("edges_sharpness_threshold", edgesSharpnessThreshold_, 0.0)) {
    ROS_WARN("did not find edges_sharpness_treshold, using default");
  }
  if (!nodeHandle_.param("max_allowed_blind_time", maxAllowedBlindTime_, 0.0)) {
    ROS_WARN("did not find max_allowed_blind_time, using default");
  }
  bool trackUnknownSpace = layered_costmap_->isTrackingUnknown();
  if (!nodeHandle_.param("track_unknown_space", trackUnknownSpace, false)) {
    ROS_WARN("did not find trackUnknownSpace, using default");
  }
  default_value_ = trackUnknownSpace ? NO_INFORMATION : FREE_SPACE;
  std::string combinationMethod;
  if (!nodeHandle_.param("combination_method", combinationMethod, std::string(""))) {
    ROS_WARN("did not find combination_method, using default");
  }
  combinationMethod_ = convertCombinationMethod(combinationMethod);

  // Subscribe to topic.
  elevationSubscriber_ = nodeHandle_.subscribe(elevationTopic_, 1, &ElevationLayer::elevationMapCallback, this);
  dsrv_ = nullptr;
  setupDynamicReconfigure(nodeHandle_);

  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle_)) {
    ROS_WARN("Could not configure the filter chain!");
  } else {
    filtersConfigurationLoaded_ = true;
  }
}

void ElevationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                  double* max_y) {
  std::lock_guard<std::mutex> lock(elevationMapMutex_);
  if (rollingWindow_) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }
  if (!(enabled_ && elevationMapReceived_)) {
    return;
  }
  useExtraBounds(min_x, min_y, max_x, max_y);

  for (grid_map::GridMapIterator iterator(elevationMap_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridmapIndex(*iterator);
    grid_map::Position vertexPositionXY;
    elevationMap_.getPosition(gridmapIndex, vertexPositionXY);
    double px = vertexPositionXY.x();
    double py = vertexPositionXY.y();

    touch(px, py, min_x, min_y, max_x, max_y);
  }
  updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
}

void ElevationLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                                     double* max_y) {
  if (!footprintClearingEnabled_) {
    return;
  }
  costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformedFootprint_);

  for (auto& i : transformedFootprint_) {
    touch(i.x, i.y, min_x, min_y, max_x, max_y);
  }
}

void ElevationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j) {
  std::lock_guard<std::mutex> lock(elevationMapMutex_);
  if (!enabled_ || !elevationMapReceived_) {
    return;
  }
  const bool hasEdgesLayer = elevationMap_.exists(edgesLayerName_);
  if (!hasEdgesLayer) {
    ROS_WARN_STREAM_THROTTLE(0.5, edgesLayerName_ << " layer not found !!");
  }
  ros::Duration timeSinceElevationMapReceived = ros::Time::now() - lastElevationMapUpdate_;
  if (timeSinceElevationMapReceived > ros::Duration(maxAllowedBlindTime_)) {
    current_ = false;
  }
  const grid_map::Matrix& elevationData = elevationMap_[elevationLayerName_];
  for (grid_map::GridMapIterator iterator(elevationMap_); !iterator.isPastEnd(); ++iterator) {
    const grid_map::Index gridmapIndex(*iterator);
    grid_map::Position vertexPositionXY;
    elevationMap_.getPosition(gridmapIndex, vertexPositionXY);
    double px = vertexPositionXY.x();
    double py = vertexPositionXY.y();
    // Now we need to compute the map coordinates for the observation.
    unsigned int mx, my;
    if (!worldToMap(px, py, mx, my))  // If point outside of local costmap, ignore.
    {
      continue;
    }
    if (elevationData(gridmapIndex(0), gridmapIndex(1)) > heightThreshold_)  // If point too high, it could be an obstacle.
    {
      if (hasEdgesLayer) {
        const grid_map::Matrix& edgesData = elevationMap_[edgesLayerName_];
        if (edgesData(gridmapIndex(0), gridmapIndex(1)) < edgesSharpnessThreshold_)  // If area not sharp, dont label as obstacle.
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

  if (footprintClearingEnabled_) {
    setConvexPolygonCost(transformedFootprint_, costmap_2d::FREE_SPACE);
  }

  switch (combinationMethod_) {
    case Overwrite:
      updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
      break;
    case Maximum:
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
      break;
    default:  // Do Nothing.
      break;
  }
}

CombinationMethod convertCombinationMethod(const std::string& str) {
  if (boost::iequals(str, "Overwrite")) {  // Case insensitive comparison.
    return Overwrite;
  } else if (boost::iequals(str, "Maximum")) {  // Case insensitive comparison.
    return Maximum;
  } else {
    ROS_WARN_THROTTLE(0.5, "Unknow combination method !");
    return Unknown;
  }
}

void ElevationLayer::elevationMapCallback(const grid_map_msgs::GridMapConstPtr& elevation) {
  grid_map::GridMap incomingMap;
  grid_map::GridMap filteredMap;
  if (!grid_map::GridMapRosConverter::fromMessage(*elevation, incomingMap)) {
    ROS_WARN_THROTTLE(0.5, "Grid Map msg Conversion failed !");
    return;
  }
  lastElevationMapUpdate_ = ros::Time::now();
  incomingMap.convertToDefaultStartIndex();
  if (!(globalFrame_ == incomingMap.getFrameId())) {
    ROS_WARN_THROTTLE(0.5, "Incoming elevation_map frame different than expected! ");
  }
  // Apply filter chain.
  if (filtersConfigurationLoaded_ && filterChain_.update(incomingMap, filteredMap)) {
    std::lock_guard<std::mutex> lock(elevationMapMutex_);
    elevationMap_ = filteredMap;
    heightThreshold_ /= 2.0;  // Half the threshold since the highest sharpness is at midheigth of the obstacles.
  } else {
    std::lock_guard<std::mutex> lock(elevationMapMutex_);
    ROS_WARN_THROTTLE(0.5, "Could not use the filter chain!");
    elevationMap_ = incomingMap;
  }
  if (!elevationMapReceived_) {
    elevationMapReceived_ = true;
  }
}

void ElevationLayer::setupDynamicReconfigure(ros::NodeHandle& nodeHandle_) {
  dsrv_.reset(new dynamic_reconfigure::Server<elevation_mapping_costmap_2d_plugin::ElevationPluginConfig>(nodeHandle_));
  dynamic_reconfigure::Server<elevation_mapping_costmap_2d_plugin::ElevationPluginConfig>::CallbackType cb =
      boost::bind(&ElevationLayer::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);
}

void ElevationLayer::reconfigureCB(elevation_mapping_costmap_2d_plugin::ElevationPluginConfig& config, uint32_t level) {
  enabled_ = config.enabled;
}

void ElevationLayer::reset() {
  deactivate();
  resetMaps();
  current_ = true;
  activate();
}

void ElevationLayer::activate() {
  // if we're stopped we need to re-subscribe to topics
  elevationSubscriber_ = nodeHandle_.subscribe(elevationTopic_, 1, &ElevationLayer::elevationMapCallback, this);
}
void ElevationLayer::deactivate() { elevationSubscriber_.shutdown(); }
}  // namespace elevation_mapping_costmap_2d_plugin