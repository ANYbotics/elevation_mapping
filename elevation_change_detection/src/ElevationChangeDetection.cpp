/*
 * ElevationChangeDetection.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_change_detection/ElevationChangeDetection.hpp"

#include <grid_map_msgs/GetGridMap.h>

// Eigen
#include <Eigen/Dense>

using namespace Eigen;
using namespace grid_map;

namespace elevation_change_detection {

ElevationChangeDetection::ElevationChangeDetection(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      layer_("elevation")
{
  ROS_INFO("Elevation change detection node started.");

  readParameters();

  submapClient_ = nodeHandle_.serviceClient<grid_map_msgs::GetGridMap>(submapServiceName_);
  elevationChangePublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_change_map", 1, true);
  groundTruthPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("ground_truth_map", 1, true);
  obstacleDetectionService_ = nodeHandle_.advertiseService("detect_obstacle", &ElevationChangeDetection::detectObstacle, this);

  if (!updateDuration_.isZero()) {
    updateTimer_ = nodeHandle_.createTimer(updateDuration_, &ElevationChangeDetection::updateTimerCallback, this);
  } else {
    ROS_INFO("ElevationChangeDetection: Update rate is zero.");
  }

  requestedMapTypes_.push_back(layer_);
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
  if (updateRate != 0.0) {
    updateDuration_.fromSec(1.0 / updateRate);
  } else {
    updateDuration_.fromSec(0.0);
  }

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

  std::string bagTopicName_, pathToBag_;
  nodeHandle_.param("bag_topic_name", bagTopicName_, std::string("grid_map"));
  nodeHandle_.param("path_to_bag", pathToBag_, std::string("ground_truth.bag"));
  loadElevationMap(pathToBag_, bagTopicName_);
  if (!groundTruthMap_.exists(layer_)) ROS_ERROR("Can't find bag or topic of the ground truth map!");

  return true;
}

bool ElevationChangeDetection::loadElevationMap(const std::string& pathToBag, const std::string& topicName)
{
  return grid_map::GridMapRosConverter::loadFromBag(pathToBag, topicName, groundTruthMap_);
}

void ElevationChangeDetection::updateTimerCallback(const ros::TimerEvent& timerEvent)
{
  grid_map_msgs::GridMap mapMessage;
  ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
  submapClient_.waitForExistence();
  ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
  if (getGridMap(mapMessage)) {
    grid_map::GridMap elevationMap;
    grid_map::GridMapRosConverter::fromMessage(mapMessage, elevationMap);
    computeElevationChange(elevationMap);

    // Publish elevation change map.
    if (!publishElevationChangeMap(elevationMap)) ROS_DEBUG("Elevation change map has not been broadcasted.");
    if (!publishGroundTruthMap(groundTruthMap_)) ROS_DEBUG("Ground truth map has not been broadcasted.");
  } else {
    ROS_WARN("Failed to retrieve elevation grid map.");
  }
}

bool ElevationChangeDetection::getGridMap(grid_map_msgs::GridMap& map)
{
  submapPoint_.header.stamp = ros::Time(0);
  geometry_msgs::PointStamped submapPointTransformed;

  try {
    transformListener_.transformPoint(mapFrameId_, submapPoint_, submapPointTransformed);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  grid_map_msgs::GetGridMap submapService;
  submapService.request.position_x = submapPointTransformed.point.x;
  submapService.request.position_y = submapPointTransformed.point.y;
  submapService.request.length_x = mapLength_.x();
  submapService.request.length_y = mapLength_.y();
  submapService.request.layers.resize(requestedMapTypes_.size());

  for (unsigned int i = 0; i < requestedMapTypes_.size(); ++i) {
    submapService.request.layers[i] = requestedMapTypes_[i];
  }

  if (!submapClient_.call(submapService)) return false;
  map = submapService.response.map;
  return true;
}

void ElevationChangeDetection::computeElevationChange(grid_map::GridMap& elevationMap)
{
  elevationMap.add("elevation_change", elevationMap.get(layer_));
  elevationMap.clear("elevation_change");

  for (GridMapIterator iterator(elevationMap);
      !iterator.isPastEnd(); ++iterator) {
    // Check if elevation map has valid value
    if (!elevationMap.isValid(*iterator, layer_)) continue;
    double height = elevationMap.at(layer_, *iterator);

    // Get the ground truth height
    Vector2d position, groundTruthPosition;
    Array2i groundTruthIndex;
    elevationMap.getPosition(*iterator, position);
    groundTruthMap_.getIndex(position, groundTruthIndex);
    if (!groundTruthMap_.isValid(groundTruthIndex, layer_)) continue;
    double groundTruthHeight = groundTruthMap_.at(layer_, groundTruthIndex);

    // Add to elevation change map
    double diffElevation = std::abs(height - groundTruthHeight);
    if (diffElevation <= threshold_) continue;
    elevationMap.at("elevation_change", *iterator) = diffElevation;
  }
}

bool ElevationChangeDetection::detectObstacle(elevation_change_msgs::DetectObstacle::Request& request, elevation_change_msgs::DetectObstacle::Response& response)
{
  const int nPoses = request.path.poses.poses.size();
  if (nPoses == 0) {
    ROS_WARN("ElevationChangeDetection: No path available to check for obstacles!");
    return false;
  }
  // Get boundaries of submap.
  double margin;
  if (request.path.radius != 0.0) {
    margin = request.path.radius;
  } else {
    margin = 0.5;
  }
  double lowX, lowY, highX, highY;
  lowX = request.path.poses.poses[0].position.x - margin;
  highX = lowX + 2 * margin;
  lowY = request.path.poses.poses[0].position.y - margin;
  highY = lowY + 2 * margin;
  for (unsigned int i = 1; i < nPoses; ++i) {
    if (request.path.poses.poses[i].position.x + margin > highX) highX = request.path.poses.poses[i].position.x + margin;
    if (request.path.poses.poses[i].position.y + margin > highY) highY = request.path.poses.poses[i].position.y + margin;
    if (request.path.poses.poses[i].position.x - margin < lowX) lowX = request.path.poses.poses[i].position.x - margin;
    if (request.path.poses.poses[i].position.y - margin < lowY) lowY = request.path.poses.poses[i].position.y - margin;
  }
  grid_map_msgs::GetGridMap submapService;
  submapService.request.position_x = (highX + lowX) / 2.0;
  submapService.request.position_y = (highY + lowY) / 2.0;
  submapService.request.length_x = highX - lowX;
  submapService.request.length_y = highY - lowY;
  submapService.request.layers.resize(requestedMapTypes_.size());
  ROS_INFO_STREAM("ElevationChangeDetection: Requested map size x: " << lowX << " to " << highX);
  ROS_INFO_STREAM("ElevationChangeDetection: Requested map size y: " << lowY << " to " << highY);

  for (unsigned int i = 0; i < requestedMapTypes_.size(); ++i) {
    submapService.request.layers[i] = requestedMapTypes_[i];
  }

  if (!submapClient_.call(submapService)) {
    ROS_WARN("ElevationChangeDetection: Cannot get elevation submap!");
    return false;
  }
//  map = submapService.response.map;
  return true;
}

bool ElevationChangeDetection::publishElevationChangeMap(const grid_map::GridMap& map)
{
  if (elevationChangePublisher_.getNumSubscribers() < 1) return false;
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);
  elevationChangePublisher_.publish(message);
  ROS_DEBUG("Elevation map raw has been published.");
  return true;
}

bool ElevationChangeDetection::publishGroundTruthMap(const grid_map::GridMap& map)
{
  if (groundTruthPublisher_.getNumSubscribers() < 1) return false;
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);
  groundTruthPublisher_.publish(message);
  ROS_DEBUG("Ground truth map raw has been published.");
  return true;
}

} /* namespace elevation_change_detection */
