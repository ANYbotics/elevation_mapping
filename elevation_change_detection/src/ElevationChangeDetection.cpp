/*
 * ElevationChangeDetection.cpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Martin Wermelinger, Peter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_change_detection/ElevationChangeDetection.hpp"

#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GetGridMap.h>

// Eigen
#include <Eigen/Dense>

using namespace Eigen;
using namespace grid_map;

namespace elevation_change_detection {

ElevationChangeDetection::ElevationChangeDetection(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      layer_("elevation"),
      elevationChangeLayer_("elevation_change")
{
  ROS_INFO("Elevation change detection node started.");

  readParameters();

  // Subscriber.
  mapSubscriber_ = nodeHandle_.subscribe(mapTopic_, 1, &ElevationChangeDetection::mapCallback, this);

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
  nodeHandle_.param("update_rate", updateRate, 0.0);
  if (updateRate != 0.0) {
    updateDuration_.fromSec(1.0 / updateRate);
  } else {
    updateDuration_.fromSec(0.0);
  }

  nodeHandle_.param("map_frame_id", mapFrameId_, std::string("map"));
  nodeHandle_.param("robot_frame_id", robotFrameId_, std::string("base"));
  double mapCenterX, mapCenterY;
  nodeHandle_.param("map_center_x", mapCenterX, 0.0);
  nodeHandle_.param("map_center_y", mapCenterY, 0.0);
  submapPoint_.header.frame_id = robotFrameId_;
  submapPoint_.point.x = mapCenterX;
  submapPoint_.point.y = mapCenterY;
  submapPoint_.point.z = 0.0;

  nodeHandle_.param("map_length_x", mapLength_.x(), 5.0);
  nodeHandle_.param("map_length_y", mapLength_.y(), 5.0);

  nodeHandle_.param("threshold", threshold_, 0.0);
  nodeHandle_.param("min_cell_number_obstacle", minNumberAdjacentCells_, 0);
  nodeHandle_.param("min_cell_number_unknown_area", minNumberAdjacentCellsUnknownArea_, 0);
  nodeHandle_.param("unknown_cells_horizon", unknownCellsHorizon_, 1.0);
  nodeHandle_.param("min_distance_to_unknown_cells", minDistanceToUnknownCell_, 1.0);
  nodeHandle_.param("check_for_unknown_areas", checkForUnkownAreas_, false);
  nodeHandle_.param("elevation_map_topic", mapTopic_, std::string("/elevation_map"));

  std::string bagTopicName_, pathToBag_;
  nodeHandle_.param("bag_topic_name", bagTopicName_, std::string("grid_map"));
  nodeHandle_.param("path_to_bag", pathToBag_, std::string("ground_truth.bag"));
  loadElevationMap(pathToBag_, bagTopicName_);
  if (!groundTruthMap_.exists(layer_)) ROS_ERROR_STREAM("ElevationChangeDetection: Bag file does not contain layer " << layer_ << "!");

  // Get obstacle free areas.
  XmlRpc::XmlRpcValue polygons;
  if (nodeHandle_.getParam("obstacle_free_areas", polygons)) {
    for (unsigned int i = 0; i < polygons.size(); ++i) {
      if (polygons[i].size() < 3) {
        ROS_WARN("Obstacle free area must consist of at least 3 points. Only %i points found.", polygons[i].size());
        continue;
      } else {
        grid_map::Polygon polygon;
        polygon.setFrameId(mapFrameId_);
        polygon.setTimestamp(ros::Time::now().toNSec());
        for (unsigned int j = 0; j < polygons[i].size(); ++j) {
          grid_map::Position position;
          position.x() = (double) polygons[i][j][0];
          position.y() = (double) polygons[i][j][1];
          polygon.addVertex(position);
        }
        obstacleFreeAreas_.push_back(polygon);
      }
    }
  }

  return true;
}

bool ElevationChangeDetection::loadElevationMap(const std::string& pathToBag, const std::string& topicName)
{
  return grid_map::GridMapRosConverter::loadFromBag(pathToBag, topicName, groundTruthMap_);
}

void ElevationChangeDetection::updateTimerCallback(const ros::TimerEvent& timerEvent)
{
  submapPoint_.header.stamp = ros::Time(0);
  geometry_msgs::PointStamped submapPointTransformed;
  try {
    transformListener_.transformPoint(mapFrameId_, submapPoint_, submapPointTransformed);
  } catch (tf::TransformException &ex) {
    ROS_ERROR("%s", ex.what());
  }

  grid_map_msgs::GridMap mapMessage;
  grid_map::Position position(submapPointTransformed.point.x, submapPointTransformed.point.y);
  grid_map::Length length(mapLength_.x(), mapLength_.y());
  if (getGridMap(position, length, mapMessage)) {
    grid_map::GridMap elevationMap;
    grid_map::GridMapRosConverter::fromMessage(mapMessage, elevationMap);
    computeElevationChange(elevationMap);
  } else {
    ROS_WARN("Failed to retrieve elevation grid map.");
  }
}

bool ElevationChangeDetection::getGridMap(const grid_map::Position& position, const grid_map::Length& length, grid_map_msgs::GridMap& map)
{
  grid_map_msgs::GetGridMap submapService;
  submapService.request.position_x = position.x();
  submapService.request.position_y = position.y();
  submapService.request.length_x = length.x();
  submapService.request.length_y = length.y();
  submapService.request.layers = requestedMapTypes_;

  ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
  submapClient_.waitForExistence();
  ROS_DEBUG("Sending request to %s.", submapServiceName_.c_str());
  if (!submapClient_.call(submapService)) return false;
  map = submapService.response.map;
  return true;
}

void ElevationChangeDetection::computeElevationChange(grid_map::GridMap& elevationMap)
{
  elevationMap.add(elevationChangeLayer_, elevationMap.get(layer_));
  elevationMap.clear(elevationChangeLayer_);

  for (GridMapIterator iterator(elevationMap);
      !iterator.isPastEnd(); ++iterator) {
    // Check if elevation map has valid value
    if (!elevationMap.isValid(*iterator, layer_)) continue;
    double height = elevationMap.at(layer_, *iterator);

    // Get the ground truth height
    Vector2d position;
    Array2i groundTruthIndex;
    elevationMap.getPosition(*iterator, position);
    if (!groundTruthMap_.getIndex(position, groundTruthIndex)) continue;
    if (!groundTruthMap_.isValid(groundTruthIndex, layer_)) continue;
    double groundTruthHeight = groundTruthMap_.at(layer_, groundTruthIndex);

    // Add to elevation change map
    double diffElevation = height - groundTruthHeight;
    if (std::abs(diffElevation) <= threshold_) continue;
    elevationMap.at(elevationChangeLayer_, *iterator) = diffElevation;
  }

  // Publish elevation change map.
  if (!publishElevationChangeMap(elevationMap)) ROS_DEBUG("Elevation change map has not been broadcasted.");
  if (!publishGroundTruthMap(groundTruthMap_)) ROS_DEBUG("Ground truth map has not been broadcasted.");
}

bool ElevationChangeDetection::detectObstacle(elevation_change_msgs::DetectObstacle::Request& request, elevation_change_msgs::DetectObstacle::Response& response)
{
  const int nPaths = request.path.size();
  if (nPaths == 0) {
    ROS_WARN("ElevationChangeDetection: No path available to check for obstacles!");
    return false;
  }

  // Set elevation map.
  grid_map::GridMap elevationMap = currentElevationMap_;
  computeElevationChange(elevationMap);
  traversability_msgs::FootprintPath path;
  for (int i = 0; i < nPaths; i++) {
    path = request.path[i];
    std::vector<elevation_change_msgs::Obstacle> obstacles;
    elevation_change_msgs::ObstacleResult result;
    if (!checkPathForObstacles(path, elevationMap, obstacles)) return false;
    if (i == nPaths - 1 && checkForUnkownAreas_)
    {
      checkPathForUnknownAreas(path, elevationMap, obstacles);
      // TODO: add obstacle to list (after testing).
    }
    for (int j = 0; j < obstacles.size(); ++j) {
      result.obstacles.push_back(obstacles[j]);
    }
    response.obstacles.push_back(result);
  }
  return true;
}

bool ElevationChangeDetection::checkPathForObstacles(const traversability_msgs::FootprintPath& path, grid_map::GridMap elevationMap, std::vector<elevation_change_msgs::Obstacle>& obstacles)
{
  const int nPoses = path.poses.poses.size();
  if (nPoses == 0) {
    ROS_WARN("ElevationChangeDetection: No poses available on the path to check for obstacles!");
    return false;
  }

  // Check path for obstacles.
  elevationMap.add("inquired_cells");
  double radius = path.radius;
  grid_map::Polygon polygon;
  grid_map::Position start, end;

  if (path.footprint.polygon.points.size() == 0) {
    for (int i = 0; i < nPoses; i++) {
      start = end;
      end.x() = path.poses.poses[i].position.x;
      end.y() = path.poses.poses[i].position.y;

      if (nPoses == 1) {
        polygon = grid_map::Polygon::fromCircle(end, radius);
        checkPolygonForObstacles(polygon, elevationMap, obstacles);
      }

      if (nPoses > 1 && i > 0) {
        polygon = grid_map::Polygon::convexHullOfTwoCircles(start, end, radius);
        checkPolygonForObstacles(polygon, elevationMap, obstacles);
      }
    }
  } else {
    grid_map::Polygon polygon1, polygon2;
    polygon1.setFrameId(mapFrameId_);
    polygon2.setFrameId(mapFrameId_);
    for (int i = 0; i < nPoses; i++) {
      polygon1 = polygon2;
      start = end;
      polygon2.removeVertices();
      grid_map::Position3 positionToVertex, positionToVertexTransformed;
      Eigen::Translation<double, 3> toPosition;
      Eigen::Quaterniond orientation;

      toPosition.x() = path.poses.poses[i].position.x;
      toPosition.y() = path.poses.poses[i].position.y;
      toPosition.z() = path.poses.poses[i].position.z;
      orientation.x() = path.poses.poses[i].orientation.x;
      orientation.y() = path.poses.poses[i].orientation.y;
      orientation.z() = path.poses.poses[i].orientation.z;
      orientation.w() = path.poses.poses[i].orientation.w;
      end.x() = toPosition.x();
      end.y() = toPosition.y();

      for (const auto& point : path.footprint.polygon.points) {
        positionToVertex.x() = point.x;
        positionToVertex.y() = point.y;
        positionToVertex.z() = point.z;
        positionToVertexTransformed = toPosition * orientation * positionToVertex;

        grid_map::Position vertex;
        vertex.x() = positionToVertexTransformed.x();
        vertex.y() = positionToVertexTransformed.y();
        polygon2.addVertex(vertex);
      }

      if (path.conservative && i > 0) {
        grid_map::Vector startToEnd = end - start;
        std::vector<grid_map::Position> vertices1 = polygon1.getVertices();
        std::vector<grid_map::Position> vertices2 = polygon2.getVertices();
        for (const auto& vertex : vertices1) {
          polygon2.addVertex(vertex + startToEnd);
        }
        for (const auto& vertex : vertices2) {
          polygon1.addVertex(vertex - startToEnd);
        }
      }

      if (nPoses == 1) {
        polygon = polygon2;
        checkPolygonForObstacles(polygon, elevationMap, obstacles);
      }

      if (nPoses > 1 && i > 0) {
        polygon = grid_map::Polygon::convexHull(polygon1, polygon2);
        checkPolygonForObstacles(polygon, elevationMap, obstacles);
      }
    }
  }
  ROS_DEBUG_STREAM("ElevationChangeDetection: detectObstacle: Number of obstacles within all polygons: " << obstacles.size());

  return true;
}

bool ElevationChangeDetection::checkPolygonForObstacles(
    const grid_map::Polygon& polygon, grid_map::GridMap& map,
    std::vector<elevation_change_msgs::Obstacle>& obstacles)
{
  for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd();
      ++iterator) {
    // Check if cell already inquired.
    if (map.isValid(*iterator, "inquired_cells")) {
      continue;
    }
    if (!map.isValid(*iterator, elevationChangeLayer_)) {
      map.at("inquired_cells", *iterator) = 0.0;
      continue;
    }
    // New obstacle detected.
    ROS_DEBUG_STREAM(
        "ElevationChangeDetection: checkPolygonForObstacles: New obstacle detected.");
    elevation_change_msgs::Obstacle obstacle;
    grid_map::Position3 obstaclePosition;
    map.getPosition3(layer_, *iterator, obstaclePosition);
    // Get size of obstacle by inquiring neighbor cells.
    map.at("inquired_cells", *iterator) = 1.0;
    std::vector<grid_map::Index> indexList;
    std::vector<grid_map::Index> inquireList;
    indexList.push_back(*iterator);
    inquireList.push_back(*iterator);

    // Populate list with cells that belong to the obstacle.
    while (!inquireList.empty()) {
      grid_map::Index index = inquireList.back();
      inquireList.pop_back();
      // Get neighbor cells.
      grid_map::Length subMapLength(3.0 * map.getResolution(),
                                    3.0 * map.getResolution());
      grid_map::Position subMapPos;
      bool isSuccess;
      map.getPosition(index, subMapPos);
      grid_map::GridMap subMap = map.getSubmap(subMapPos, subMapLength,
                                               isSuccess);
      if (!isSuccess) {
        ROS_WARN(
            "ElevationChangeDetection: checkPolygonForObstacles: Could not retrieve submap.");
        map.at("inquired_cells", index) = 0.0;
        continue;
      }

      for (grid_map::GridMapIterator subMapIterator(subMap);
          !subMapIterator.isPastEnd(); ++subMapIterator) {
        if (subMap.isValid(*subMapIterator, "inquired_cells")) {
          continue;
        }
        // Get index in map.
        grid_map::Position pos;
        subMap.getPosition(*subMapIterator, pos);
        grid_map::Index mapIndex;
        map.getIndex(pos, mapIndex);
        if (!subMap.isValid(*subMapIterator, elevationChangeLayer_)) {
          map.at("inquired_cells", mapIndex) = 0.0;
          continue;
        } else {
          map.at("inquired_cells", mapIndex) = 1.0;
          indexList.push_back(mapIndex);
          inquireList.push_back(mapIndex);
        }
      }
    }

    if (indexList.size() < minNumberAdjacentCells_)
      continue;
    // Compute size of obstacle.
    double minX = obstaclePosition.x();
    double minY = obstaclePosition.y();
    double maxX = obstaclePosition.x();
    double maxY = obstaclePosition.y();
    double maxHeight = 0;
    obstaclePosition.setZero();
    grid_map::Position3 point;
    unsigned int nCells = indexList.size();
    for (unsigned int i = 0; i < nCells; ++i) {
      map.getPosition3(layer_, indexList[i], point);
      obstaclePosition += point;
      minX = std::min(minX, point.x());
      maxX = std::max(maxX, point.x());
      minY = std::min(minY, point.y());
      maxY = std::max(maxY, point.y());
      double elevationChange = map.at(elevationChangeLayer_, indexList[i]);
      if (std::abs(elevationChange) > std::abs(maxHeight))
        maxHeight = elevationChange;
    }
    obstaclePosition /= nCells;

    // Classify // TODO: type "supended"
    if (maxHeight > 0) {
      obstacle.type = "positive";
    } else {
      obstacle.type = "negative";
    }
    obstacle.length = maxX - minX;
    obstacle.width = maxY - minY;
    obstacle.height = maxHeight;
    obstacle.pose.position.x = obstaclePosition.x();
    obstacle.pose.position.y = obstaclePosition.y();
    obstacle.pose.position.z = obstaclePosition.z();
    // Reject obstacles within defined area.
    grid_map::Position position;
    position.x() = obstaclePosition.x();
    position.y() = obstaclePosition.y();
    for (unsigned int i = 0; i < obstacleFreeAreas_.size(); ++i) {
      if (obstacleFreeAreas_.at(i).isInside(position)) continue;
    }
    obstacles.push_back(obstacle);
  }
  return true;
}

bool ElevationChangeDetection::checkPathForUnknownAreas(
    const traversability_msgs::FootprintPath& path, grid_map::GridMap map,
    std::vector<elevation_change_msgs::Obstacle>& obstacles)
{
  // Get current pose.
  geometry_msgs::PoseStamped currentPose = getCurrentPose();
  // Get poses to verify.
  traversability_msgs::FootprintPath localPath;
  localPath.poses.header = path.poses.header;
  for (unsigned int i = 1; i < path.poses.poses.size(); ++i) {
    double distance = std::pow((currentPose.pose.position.x - path.poses.poses[i].position.x), 2) + std::pow((currentPose.pose.position.y - path.poses.poses[i].position.y), 2);
    distance = std::sqrt(distance);
    if (distance > unknownCellsHorizon_) break;
    localPath.poses.poses.push_back(path.poses.poses[i]);
  }

  unsigned int nPoses = localPath.poses.poses.size();
  map.add("inquired_cells");
  double radius = path.radius;
  grid_map::Polygon polygon;
  grid_map::Position start, end;

  if (path.footprint.polygon.points.size() == 0) {
    for (int i = 0; i < nPoses; i++) {
      start = end;
      end.x() = path.poses.poses[i].position.x;
      end.y() = path.poses.poses[i].position.y;

      if (nPoses == 1) {
        polygon = grid_map::Polygon::fromCircle(end, radius);
        checkPolygonForUnknownAreas(polygon, currentPose, map, obstacles);
      }

      if (nPoses > 1 && i > 0) {
        polygon = grid_map::Polygon::convexHullOfTwoCircles(start, end, radius);
        checkPolygonForUnknownAreas(polygon, currentPose, map, obstacles);
      }
    }
  } else {
    grid_map::Polygon polygon1, polygon2;
    polygon1.setFrameId(mapFrameId_);
    polygon2.setFrameId(mapFrameId_);
    for (int i = 0; i < nPoses; i++) {
      polygon1 = polygon2;
      start = end;
      polygon2.removeVertices();
      grid_map::Position3 positionToVertex, positionToVertexTransformed;
      Eigen::Translation<double, 3> toPosition;
      Eigen::Quaterniond orientation;

      toPosition.x() = path.poses.poses[i].position.x;
      toPosition.y() = path.poses.poses[i].position.y;
      toPosition.z() = path.poses.poses[i].position.z;
      orientation.x() = path.poses.poses[i].orientation.x;
      orientation.y() = path.poses.poses[i].orientation.y;
      orientation.z() = path.poses.poses[i].orientation.z;
      orientation.w() = path.poses.poses[i].orientation.w;
      end.x() = toPosition.x();
      end.y() = toPosition.y();

      for (const auto& point : path.footprint.polygon.points) {
        positionToVertex.x() = point.x;
        positionToVertex.y() = point.y;
        positionToVertex.z() = point.z;
        positionToVertexTransformed = toPosition * orientation * positionToVertex;

        grid_map::Position vertex;
        vertex.x() = positionToVertexTransformed.x();
        vertex.y() = positionToVertexTransformed.y();
        polygon2.addVertex(vertex);
      }

      if (path.conservative && i > 0) {
        grid_map::Vector startToEnd = end - start;
        std::vector<grid_map::Position> vertices1 = polygon1.getVertices();
        std::vector<grid_map::Position> vertices2 = polygon2.getVertices();
        for (const auto& vertex : vertices1) {
          polygon2.addVertex(vertex + startToEnd);
        }
        for (const auto& vertex : vertices2) {
          polygon1.addVertex(vertex - startToEnd);
        }
      }

      if (nPoses == 1) {
        polygon = polygon2;
        checkPolygonForUnknownAreas(polygon, currentPose, map, obstacles);
      }

      if (nPoses > 1 && i > 0) {
        polygon = grid_map::Polygon::convexHull(polygon1, polygon2);
        checkPolygonForUnknownAreas(polygon, currentPose, map, obstacles);
      }
    }
  }


  return true;
}

bool ElevationChangeDetection::checkPolygonForUnknownAreas(const grid_map::Polygon& polygon, const geometry_msgs::PoseStamped& currentPose, grid_map::GridMap& map, std::vector<elevation_change_msgs::Obstacle>& obstacles)
{
  ROS_DEBUG_STREAM("ElevationChangeDetection: checkPolygonForUnknownAreas");
  for (grid_map::PolygonIterator iterator(map, polygon); !iterator.isPastEnd();
      ++iterator) {
    // Check if cell already inquired.
    if (map.isValid(*iterator, "inquired_cells")) {
      continue;
    }
    // Continue if cell is valid.
    if (map.isValid(*iterator, layer_)) {
      map.at("inquired_cells", *iterator) = 0.0;
      continue;
    }
    elevation_change_msgs::Obstacle obstacle;
    grid_map::Position obstaclePosition;
    map.getPosition(*iterator, obstaclePosition);
    // Get size of obstacle by inquiring neighbor cells.
    map.at("inquired_cells", *iterator) = 1.0;
    std::vector<grid_map::Index> indexList;
    std::vector<grid_map::Index> inquireList;
    indexList.push_back(*iterator);
    inquireList.push_back(*iterator);

    // Populate list with cells that belong to the obstacle.
    while (!inquireList.empty()) {
      grid_map::Index index = inquireList.back();
      inquireList.pop_back();
      // Get neighbor cells.
      grid_map::Length subMapLength(3.0 * map.getResolution(),
                                    3.0 * map.getResolution());
      grid_map::Position subMapPos;
      bool isSuccess;
      map.getPosition(index, subMapPos);
      grid_map::GridMap subMap = map.getSubmap(subMapPos, subMapLength,
                                               isSuccess);
      if (!isSuccess) {
        ROS_WARN_STREAM(
            "ElevationChangeDetection: checkPolygonForUnknownAreas: Could not retrieve submap at: " << subMapPos);
        map.at("inquired_cells", index) = 0.0;
        continue;
      }

      for (grid_map::GridMapIterator subMapIterator(subMap);
          !subMapIterator.isPastEnd(); ++subMapIterator) {
        if (subMap.isValid(*subMapIterator, "inquired_cells")) {
          continue;
        }
        // Get index in map.
        grid_map::Position pos;
        subMap.getPosition(*subMapIterator, pos);
        grid_map::Index mapIndex;
        map.getIndex(pos, mapIndex);
        double distance = std::pow(currentPose.pose.position.x - pos.x(), 2) + std::pow(currentPose.pose.position.y - pos.y(), 2);
        distance = std::sqrt(distance);
        if (subMap.isValid(*subMapIterator, layer_)) {
          map.at("inquired_cells", mapIndex) = 0.0;
          continue;
        } else if (distance < minDistanceToUnknownCell_ || distance > unknownCellsHorizon_) {
          map.at("inquired_cells", mapIndex) = 0.0;
          continue;
        } else {
          map.at("inquired_cells", mapIndex) = 1.0;
          indexList.push_back(mapIndex);
          inquireList.push_back(mapIndex);
        }
      }
    }

    if (indexList.size() < minNumberAdjacentCellsUnknownArea_)
      continue;
    // Compute size of obstacle.
    double minX = obstaclePosition.x();
    double minY = obstaclePosition.y();
    double maxX = obstaclePosition.x();
    double maxY = obstaclePosition.y();
    obstaclePosition.setZero();
    grid_map::Position pos;
    double obstaclePositionZ = 0;
    unsigned int nCellsWithHeightValue = 0;
    unsigned int nCells = indexList.size();
    for (unsigned int i = 0; i < nCells; ++i) {
      map.getPosition(indexList[i], pos);
      obstaclePosition += pos;
      double groundTruthHeight = groundTruthMap_.atPosition(layer_, pos);
      if (std::isfinite(groundTruthHeight)) {
        obstaclePositionZ += groundTruthHeight;
        nCellsWithHeightValue++;
      }
      minX = std::min(minX, pos.x());
      maxX = std::max(maxX, pos.x());
      minY = std::min(minY, pos.y());
      maxY = std::max(maxY, pos.y());
    }
    obstaclePosition /= nCells;
    obstaclePositionZ /= nCellsWithHeightValue;
    obstacle.type = "negative";

    obstacle.length = maxX - minX;
    obstacle.width = maxY - minY;
    obstacle.height = -0.1; //TODO
    obstacle.pose.position.x = obstaclePosition.x();
    obstacle.pose.position.y = obstaclePosition.y();
    obstacle.pose.position.z = obstaclePositionZ;
    // Reject obstacles within defined area.
    for (unsigned int i = 0; i < obstacleFreeAreas_.size(); ++i) {
      if (obstacleFreeAreas_.at(i).isInside(obstaclePosition)) continue;
    }
    obstacles.push_back(obstacle);
    ROS_INFO_STREAM("ElevationChangeDetection: checkPolygonForUnknownAreas: obstacle: " << obstacle);
  }
  return true;
}

geometry_msgs::PoseStamped ElevationChangeDetection::getCurrentPose() const
{
  geometry_msgs::PoseStamped pose;
  // Get the current pose of the robot by listening to tf.
  tf::StampedTransform transformationRobotToMap;
  transformListener_.lookupTransform(mapFrameId_, robotFrameId_, ros::Time(0), transformationRobotToMap);

  pose.header.frame_id = transformationRobotToMap.frame_id_;
  pose.header.seq = 0;
  pose.header.stamp = transformationRobotToMap.stamp_;
  pose.pose.position.x = transformationRobotToMap.getOrigin().getX();
  pose.pose.position.y = transformationRobotToMap.getOrigin().getY();
  pose.pose.position.z = transformationRobotToMap.getOrigin().getZ();
  pose.pose.orientation.w = transformationRobotToMap.getRotation().getW();
  pose.pose.orientation.x = transformationRobotToMap.getRotation().getX();
  pose.pose.orientation.y = transformationRobotToMap.getRotation().getY();
  pose.pose.orientation.z = transformationRobotToMap.getRotation().getZ();
  return pose;
}

void ElevationChangeDetection::mapCallback(const grid_map_msgs::GridMap& map)
{
  grid_map::GridMapRosConverter::fromMessage(map, currentElevationMap_);
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
