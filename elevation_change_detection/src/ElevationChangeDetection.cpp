/*
 * ElevationChangeDetection.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Martin Wermelinger
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_change_detection/ElevationChangeDetection.hpp"

#include <grid_map_lib/GridMap.hpp>
#include <grid_map_lib/GridMapMath.hpp>
#include <grid_map_lib/iterators/GridMapIterator.hpp>
#include <grid_map_msg/GetGridMap.h>

// Eigenvalues
#include <Eigen/Dense>

using namespace Eigen;

namespace elevation_change_detection {

ElevationChangeDetection::ElevationChangeDetection(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      type_("elevation")
{
  ROS_INFO("Elevation change detection node started.");

  readParameters();

  submapClient_ = nodeHandle_.serviceClient<grid_map_msg::GetGridMap>(submapServiceName_);
  elevationChangePublisher_ = nodeHandle_.advertise<grid_map_msg::GridMap>("elevation_change_map", 1, true);
  groundTruthPublisher_ = nodeHandle_.advertise<grid_map_msg::GridMap>("ground_truth_map", 1, true);

  updateTimer_ = nodeHandle_.createTimer(updateDuration_,
                                         &ElevationChangeDetection::updateTimerCallback, this);

  requestedMapTypes_.push_back(type_);
}

ElevationChangeDetection::~ElevationChangeDetection()
{
  updateTimer_.stop();
  nodeHandle_.shutdown();
}

bool ElevationChangeDetection::readParameters()
{
  nodeHandle_.param("submap_service", submapServiceName_, std::string("/get_grid_map"));

  double updateRate;
  nodeHandle_.param("update_rate", updateRate, 1.0);
  updateDuration_.fromSec(1.0 / updateRate);

  nodeHandle_.param("map_frame_id", mapFrameId_, std::string("map"));
  std::string robotFrameId;
  nodeHandle_.param("robot_frame_id", robotFrameId, std::string("base"));
  double mapCenterX, mapCenterY;
  nodeHandle_.param("map_center_x", mapCenterX, 0.0);
  nodeHandle_.param("map_center_y", mapCenterY, 0.0);
  submapPoint_.header.frame_id = robotFrameId;
  submapPoint_.point.x = mapCenterX;
  submapPoint_.point.y = mapCenterY;
  submapPoint_.point.z = 0.0;

  nodeHandle_.param("map_length_x", mapLength_.x(), 5.0);
  nodeHandle_.param("map_length_y", mapLength_.y(), 5.0);

  nodeHandle_.param("threshold", threshold_, 0.0);

  nodeHandle_.param("bag_topic_name", bagTopicName_, std::string("grid_map"));

  nodeHandle_.param("path_to_bag", pathToBag_, std::string("lee_ground_truth.bag"));
  loadElevationMap(pathToBag_, bagTopicName_);
  if (!groundTruthMap_.exists(type_)) ROS_ERROR("Can't find bag or topic of the ground truth map!");

  return true;
}

bool ElevationChangeDetection::loadElevationMap(const std::string& pathToBag, const std::string& topicName)
{
  return groundTruthMap_.loadFromBag(pathToBag, topicName);
}

void ElevationChangeDetection::updateTimerCallback(const ros::TimerEvent& timerEvent)
{
  grid_map_msg::GridMap mapMessage;
  if (getGridMap(mapMessage)) {
    grid_map::GridMap elevationMap(mapMessage);
    computeElevationChange(elevationMap);

    // Publish elevation change map.
    if (!publishElevationChangeMap(elevationMap)) ROS_DEBUG("Elevation change map has not been broadcasted.");
    if (!publishGroundTruthMap(groundTruthMap_)) ROS_DEBUG("Ground truth map has not been broadcasted.");
  } else {
    ROS_WARN("Failed to retrieve elevation grid map.");
  }
}

bool ElevationChangeDetection::getGridMap(grid_map_msg::GridMap& map)
{
  submapPoint_.header.stamp = ros::Time(0);
  geometry_msgs::PointStamped submapPointTransformed;

  try {
    transformListener_.transformPoint(mapFrameId_, submapPoint_, submapPointTransformed);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  grid_map_msg::GetGridMap submapService;
  submapService.request.positionX = submapPointTransformed.point.x;
  submapService.request.positionY = submapPointTransformed.point.y;
  submapService.request.lengthX = mapLength_.x();
  submapService.request.lengthY = mapLength_.y();
  submapService.request.dataDefinition.resize(requestedMapTypes_.size());

  for (unsigned int i = 0; i < requestedMapTypes_.size(); ++i) {
    submapService.request.dataDefinition[i] = requestedMapTypes_[i];
  }

  if (!submapClient_.call(submapService)) return false;
  map = submapService.response.gridMap;
  return true;
}

void ElevationChangeDetection::computeElevationChange(grid_map::GridMap& elevationMap)
{
  elevationMap.add("elevation_change", elevationMap.get(type_));
  std::vector<std::string> validTypes;
  std::vector<std::string> clearTypes;
  validTypes.push_back(type_);
  clearTypes.push_back("elevation_change");
  elevationMap.setClearTypes(clearTypes);
  elevationMap.clear();

  for (grid_map_lib::GridMapIterator iterator(elevationMap);
      !iterator.isPassedEnd(); ++iterator) {
    // Check if elevation map has valid value
    if (!elevationMap.isValid(*iterator, validTypes)) continue;
    double height = elevationMap.at(type_, *iterator);

    // Get the ground truth height
    Vector2d position, groundTruthPosition;
    Array2i groundTruthIndex;
    elevationMap.getPosition(*iterator, position);
    groundTruthMap_.getIndex(position, groundTruthIndex);
    if (!groundTruthMap_.isValid(groundTruthIndex, validTypes)) continue;
    double groundTruthHeight = groundTruthMap_.at(type_, groundTruthIndex);

    // Add to elevation change map
    double diffElevation = std::abs(height - groundTruthHeight);
    if (diffElevation <= threshold_) continue;
    elevationMap.at("elevation_change", *iterator) = diffElevation;
  }
}

bool ElevationChangeDetection::publishElevationChangeMap(const grid_map::GridMap& map)
{
  if (elevationChangePublisher_.getNumSubscribers() < 1) return false;
  grid_map_msg::GridMap message;
  map.toMessage(message);
  elevationChangePublisher_.publish(message);
  ROS_DEBUG("Elevation map raw has been published.");
  return true;
}

bool ElevationChangeDetection::publishGroundTruthMap(const grid_map::GridMap& map)
{
  if (groundTruthPublisher_.getNumSubscribers() < 1) return false;
  grid_map_msg::GridMap message;
  map.toMessage(message);
  groundTruthPublisher_.publish(message);
  ROS_DEBUG("Ground truth map raw has been published.");
  return true;
}

} /* namespace elevation_change_detection */
