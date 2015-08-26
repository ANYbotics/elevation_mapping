/*
 * ElevationMap.cpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#include "elevation_mapping/ElevationMap.hpp"

// Elevation Mapping
#include "elevation_mapping/ElevationMapFunctors.hpp"

// Grid Map
#include <grid_map_msgs/GridMap.h>

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

// Eigenvalues
#include <Eigen/Core>
#include <Eigen/Dense>

using namespace std;
using namespace grid_map;
using namespace sm;
using namespace sm::timing;

namespace elevation_mapping {

ElevationMap::ElevationMap(ros::NodeHandle nodeHandle)
    : nodeHandle_(nodeHandle),
      rawMap_({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "color"}),
      fusedMap_({"elevation", "variance", "color", "surface_normal_x", "surface_normal_y", "surface_normal_z"}),
      hasUnderlyingMap_(false)
{
  readParameters();
  rawMap_.setBasicLayers({"elevation", "variance"});
  fusedMap_.setBasicLayers({"elevation", "variance"});
  clear();

  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map_raw", 1);
  elevationMapFusedPublisher_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  if (!underlyingMapTopic_.empty()) underlyingMapSubscriber_ =
      nodeHandle_.subscribe(underlyingMapTopic_, 1, &ElevationMap::underlyingMapCallback, this);
}

ElevationMap::~ElevationMap()
{
}

bool ElevationMap::readParameters()
{
  string frameId;
  nodeHandle_.param("map_frame_id", frameId, string("/map"));
  setFrameId(frameId);

  grid_map::Length length;
  grid_map::Position position;
  double resolution;
  nodeHandle_.param("length_in_x", length(0), 1.5);
  nodeHandle_.param("length_in_y", length(1), 1.5);
  nodeHandle_.param("position_x", position.x(), 0.0);
  nodeHandle_.param("position_y", position.y(), 0.0);
  nodeHandle_.param("resolution", resolution, 0.01);
  setGeometry(length, resolution, position);

  nodeHandle_.param("min_variance", minVariance_, pow(0.003, 2));
  nodeHandle_.param("max_variance", maxVariance_, pow(0.03, 2));
  nodeHandle_.param("mahalanobis_distance_threshold", mahalanobisDistanceThreshold_, 2.5);
  nodeHandle_.param("multi_height_noise", multiHeightNoise_, pow(0.003, 2));
  nodeHandle_.param("min_horizontal_variance", minHorizontalVariance_, pow(resolution / 2.0, 2)); // two-sigma
  nodeHandle_.param("max_horizontal_variance", maxHorizontalVariance_, 0.5);
  nodeHandle_.param("surface_normal_estimation_radius", surfaceNormalEstimationRadius_, 0.05);
  nodeHandle_.param("underlying_map_topic", underlyingMapTopic_, string());

  string surfaceNormalPositiveAxis;
  nodeHandle_.param("surface_normal_positive_axis", surfaceNormalPositiveAxis, string("z"));
  if (surfaceNormalPositiveAxis == "z") {
    surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitZ();
  } else if (surfaceNormalPositiveAxis == "y") {
    surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitY();
  } else if (surfaceNormalPositiveAxis == "x") {
    surfaceNormalPositiveAxis_ = grid_map::Vector3::UnitX();
  } else {
    ROS_ERROR("The surface normal positive axis '%s' is not valid.", surfaceNormalPositiveAxis.c_str());
  }
}

void ElevationMap::setGeometry(const Eigen::Array2d& length, const double& resolution, const Eigen::Vector2d& position)
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  rawMap_.setGeometry(length, resolution, position);
  fusedMap_.setGeometry(length, resolution, position);
  ROS_INFO_STREAM("Elevation map grid resized to " << rawMap_.getSize()(0) << " rows and "  << rawMap_.getSize()(1) << " columns.");
}

bool ElevationMap::add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances)
{
  if (pointCloud->size() != pointCloudVariances.size()) {
    ROS_ERROR("ElevationMap::add: Size of point cloud (%i) and variances (%i) do not agree.",
              (int) pointCloud->size(), (int) pointCloudVariances.size());
    return false;
  }

  for (unsigned int i = 0; i < pointCloud->size(); ++i) {
    auto& point = pointCloud->points[i];

    Index index;
    Position position(point.x, point.y);
    if (!rawMap_.getIndex(position, index)) continue; // Skip this point if it does not lie within the elevation map.

    auto& elevation = rawMap_.at("elevation", index);
    auto& variance = rawMap_.at("variance", index);
    auto& horizontalVarianceX = rawMap_.at("horizontal_variance_x", index);
    auto& horizontalVarianceY = rawMap_.at("horizontal_variance_y", index);
    auto& color = rawMap_.at("color", index);
    const float& pointVariance = pointCloudVariances(i);

    if (!rawMap_.isValid(index))
    {
      // No prior information in elevation map, use measurement.
      elevation = point.z;
      variance = pointVariance;
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      colorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    double mahalanobisDistance = sqrt(pow(point.z - elevation, 2) / variance);

    if (mahalanobisDistance < mahalanobisDistanceThreshold_)
    {
      // Fuse measurement with elevation map data.
      elevation = (variance * point.z + pointVariance * elevation) / (variance + pointVariance);
      variance =  (pointVariance * variance) / (pointVariance + variance);
      // TODO Add color fusion.
      colorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    // Add noise to cells which have ignored lower values,
    // such that outliers and moving objects are removed.
    variance += multiHeightNoise_;

    // Horizontal variances are reset.
    horizontalVarianceX = minHorizontalVariance_;
    horizontalVarianceY = minHorizontalVariance_;
  }

  clean();
  rawMap_.setTimestamp(1000 * pointCloud->header.stamp); // Point cloud stores time in microseconds.
  return true;
}

bool ElevationMap::update(Eigen::MatrixXf varianceUpdate, Eigen::MatrixXf horizontalVarianceUpdateX,
                          Eigen::MatrixXf horizontalVarianceUpdateY, const ros::Time& time)
{
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);

  const auto& size = rawMap_.getSize();

  if (!(
      (Index(varianceUpdate.rows(), varianceUpdate.cols()) == size).all() &&
      (Index(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == size).all() &&
      (Index(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == size).all()
      ))
  {
    ROS_ERROR("The size of the update matrices does not match.");
    return false;
  }

  rawMap_.get("variance") += varianceUpdate;
  rawMap_.get("horizontal_variance_x") += horizontalVarianceUpdateX;
  rawMap_.get("horizontal_variance_y") += horizontalVarianceUpdateY;
  clean();
  rawMap_.setTimestamp(time.toNSec());

  return true;
}

bool ElevationMap::fuseAll(const bool computeSurfaceNormals)
{
  ROS_DEBUG("Requested to fuse entire elevation map.");
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return fuse(Index(0, 0), fusedMap_.getSize(), computeSurfaceNormals);
}

bool ElevationMap::fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length, const bool computeSurfaceNormals)
{
  ROS_DEBUG("Requested to fuse an area of the elevation map with center at (%f, %f) and side lengths (%f, %f)",
            position[0], position[1], length[0], length[1]);

  Index topLeftIndex;
  Index submapBufferSize;

  // These parameters are not used in this function.
  Position submapPosition;
  Length submapLength;
  Index requestedIndexInSubmap;

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength,
                       requestedIndexInSubmap, position, length, rawMap_.getLength(),
                       rawMap_.getPosition(), rawMap_.getResolution(), rawMap_.getSize(),
                       rawMap_.getStartIndex());

  return fuse(topLeftIndex, submapBufferSize, computeSurfaceNormals);
}

bool ElevationMap::fuse(const Eigen::Array2i& topLeftIndex, const Eigen::Array2i& size, const bool computeSurfaceNormals)
{
  ROS_DEBUG("Fusing elevation map...");

  // Nothing to do.
  if ((size == 0).any()) return false;

  // Initializations.
  string timerId = "map_fusion_timer";
  Timer timer(timerId, true);

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  // Copy raw elevation map data for safe multi-threading.
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  auto rawMapCopy = rawMap_;
  scopedLockForRawData.unlock();

  // Check if there is the need to reset out-dated data.
  if (fusedMap_.getTimestamp() != rawMapCopy.getTimestamp()) resetFusedData();

  // Align fused map with raw map.
  if (rawMapCopy.getPosition() != fusedMap_.getPosition()) fusedMap_.move(rawMapCopy.getPosition());

  // For each cell in requested area.
  for (SubmapIterator areaIterator(rawMapCopy, topLeftIndex, size); !areaIterator.isPastEnd(); ++areaIterator) {
    if (timer.isTiming()) timer.stop();
    timer.start();

    // Check if fusion for this cell has already been done earlier.
    if (fusedMap_.isValid(*areaIterator)) continue;

    if (!rawMapCopy.isValid(*areaIterator)) {
      // This is an empty cell (hole in the map).
      continue;
    }

    // Size of submap (2 sigma bound). TODO Add minimum/maximum submap size?
    Length requestedSubmapLength = 4.0
        * Length(rawMapCopy.at("horizontal_variance_x", *areaIterator),
                 rawMapCopy.at("horizontal_variance_y", *areaIterator)).sqrt();

    // Requested position (center) of submap in map.
    Position requestedSubmapPosition;
    rawMapCopy.getPosition(*areaIterator, requestedSubmapPosition);

    Length submapLength;
    Position submapPosition;
    Index submapTopLeftIndex, submapBufferSize, requestedIndexInSubmap;

    getSubmapInformation(submapTopLeftIndex, submapBufferSize, submapPosition, submapLength,
                         requestedIndexInSubmap, requestedSubmapPosition, requestedSubmapLength,
                         rawMapCopy.getLength(), rawMapCopy.getPosition(),
                         rawMapCopy.getResolution(), rawMapCopy.getSize(),
                         rawMapCopy.getStartIndex());

    // Prepare data fusion.
    Eigen::ArrayXf means, variances, weights;
    int maxNumberOfCellsToFuse = submapBufferSize.prod();
    means.resize(maxNumberOfCellsToFuse);
    variances.resize(maxNumberOfCellsToFuse);
    weights.resize(maxNumberOfCellsToFuse);

    // For each cell in submap.
    size_t i = 0;
    for (SubmapIterator submapIterator(rawMapCopy, submapTopLeftIndex, submapBufferSize); !submapIterator.isPastEnd(); ++submapIterator) {
      if (!rawMapCopy.isValid(*submapIterator)) {
        // Empty cell in submap (cannot be center cell because we checked above).
        continue;
      }

      means[i] = rawMapCopy.at("elevation", *submapIterator);
      variances[i] = rawMapCopy.at("variance", *submapIterator);

      // Compute weight from probability.
      Position position;
      rawMapCopy.getPosition(*submapIterator, position);

      Eigen::Vector2d distanceToCenter = (position - submapPosition).cwiseAbs();

      float probabilityX =
            cumulativeDistributionFunction(distanceToCenter.x() + rawMapCopy.getResolution() / 2.0, 0.0, sqrt(rawMapCopy.at("horizontal_variance_x", *submapIterator)))
          - cumulativeDistributionFunction(distanceToCenter.x() - rawMapCopy.getResolution() / 2.0, 0.0, sqrt(rawMapCopy.at("horizontal_variance_x", *submapIterator)));
      float probabilityY =
            cumulativeDistributionFunction(distanceToCenter.y() + rawMapCopy.getResolution() / 2.0, 0.0, sqrt(rawMapCopy.at("horizontal_variance_y", *submapIterator)))
          - cumulativeDistributionFunction(distanceToCenter.y() - rawMapCopy.getResolution() / 2.0, 0.0, sqrt(rawMapCopy.at("horizontal_variance_y", *submapIterator)));

      weights[i] = probabilityX * probabilityY;
      i++;
    }

    if (i == 0) {
      // Nothing to fuse.
      fusedMap_.at("elevation", *areaIterator) = rawMapCopy.at("elevation", *areaIterator);
      fusedMap_.at("variance", *areaIterator) = rawMapCopy.at("variance", *areaIterator);
      fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);
      continue;
    }

    // Fuse.
    means.conservativeResize(i);
    variances.conservativeResize(i);
    weights.conservativeResize(i);

    float mean = (weights * means).sum() / weights.sum();
    float variance = (weights * (variances + means.square())).sum() / weights.sum() - pow(mean, 2);

    if (!(std::isfinite(variance) && std::isfinite(mean))) {
      ROS_ERROR("Something went wrong when fusing the map: Mean = %f, Variance = %f", mean, variance);
      continue;
    }

    // Add to fused map.
    fusedMap_.at("elevation", *areaIterator) = mean;
    fusedMap_.at("variance", *areaIterator) = variance;

    // TODO Add fusion of colors.
    fusedMap_.at("color", *areaIterator) = rawMapCopy.at("color", *areaIterator);

    // Stop from data corruption with old surface normals.
    fusedMap_.at("surface_normal_x", *areaIterator) = NAN;
    fusedMap_.at("surface_normal_y", *areaIterator) = NAN;
    fusedMap_.at("surface_normal_z", *areaIterator) = NAN;

    timer.stop();
  }

  fusedMap_.setTimestamp(rawMapCopy.getTimestamp());

  ROS_INFO("Elevation map has been fused in %f s.", Timing::getTotalSeconds(timerId));
  ROS_DEBUG("Mean: %f s, Min: %f s, Max: %f s.", Timing::getMeanSeconds(timerId), Timing::getMinSeconds(timerId), Timing::getMaxSeconds(timerId));
  Timing::reset(timerId);

  if (computeSurfaceNormals) return ElevationMap::computeSurfaceNormals(topLeftIndex, size);
  return true;
}

bool ElevationMap::computeSurfaceNormals(const Eigen::Array2i& topLeftIndex, const Eigen::Array2i& size)
{
  // TODO Change this to raw map!
  ROS_DEBUG("Computing surface normals...");

  // Initializations.
  string timerId = "map_surface_normal_computation_timer";
  Timer timer(timerId, true);

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  vector<string> surfaceNormalTypes;
  surfaceNormalTypes.push_back("surface_normal_x");
  surfaceNormalTypes.push_back("surface_normal_y");
  surfaceNormalTypes.push_back("surface_normal_z");

  // For each cell in requested area.
  for (SubmapIterator areaIterator(fusedMap_, topLeftIndex, size); !areaIterator.isPastEnd(); ++areaIterator) {
    if (timer.isTiming()) timer.stop();
    timer.start();

    // Check if this is an empty cell (hole in the map).
    if (!fusedMap_.isValid(*areaIterator)) continue;
    // Check if surface normal for this cell has already been computed earlier.
    if (fusedMap_.isValid(*areaIterator, surfaceNormalTypes)) continue;

    // Size of submap area for surface normal estimation.
    Length submapLength = Length::Ones() * (2.0 * surfaceNormalEstimationRadius_);

    // Requested position (center) of submap in map.
    Position submapPosition;
    fusedMap_.getPosition(*areaIterator, submapPosition);
    Index submapTopLeftIndex, submapBufferSize, requestedIndexInSubmap;
    getSubmapInformation(submapTopLeftIndex, submapBufferSize, submapPosition, submapLength,
                         requestedIndexInSubmap, submapPosition, submapLength,
                         fusedMap_.getLength(), fusedMap_.getPosition(), fusedMap_.getResolution(),
                         fusedMap_.getSize(), fusedMap_.getStartIndex());

    // Prepare data computation.
    const int maxNumberOfCells = submapBufferSize.prod();
    Eigen::MatrixXd points(3, maxNumberOfCells);

    // Gather surrounding data.
    size_t nPoints = 0;
    for (SubmapIterator submapIterator(fusedMap_, submapTopLeftIndex, submapBufferSize); !submapIterator.isPastEnd(); ++submapIterator) {
      if (!fusedMap_.isValid(*submapIterator)) continue;
      Position3 point;
      fusedMap_.getPosition3("elevation", *submapIterator, point);
      points.col(nPoints) = point;
      nPoints++;
    }
//    nPoints.conservativeResize(3, nPoints); // TODO Eigen version?

    // Compute eigenvectors.
    const Eigen::Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
    const Eigen::MatrixXd NN = points.leftCols(nPoints).colwise() - mean;

    const Eigen::Matrix3d covarianceMatrix(NN * NN.transpose());
    Eigen::Vector3d eigenvalues = Eigen::Vector3d::Identity();
    Eigen::Matrix3d eigenvectors = Eigen::Matrix3d::Identity();
    // Ensure that the matrix is suited for eigenvalues calculation
    if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
      const Eigen::EigenSolver<Eigen::MatrixXd> solver(covarianceMatrix);
      eigenvalues = solver.eigenvalues().real();
      eigenvectors = solver.eigenvectors().real();
    } else {
      ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", (int) nPoints);
    }
    // Keep the smallest eigenvector as surface normal
    int smallestId(0);
    double smallestValue(numeric_limits<double>::max());
    for (int j = 0; j < eigenvectors.cols(); j++) {
      if (eigenvalues(j) < smallestValue) {
        smallestId = j;
        smallestValue = eigenvalues(j);
      }
    }
    Eigen::Vector3d eigenvector = eigenvectors.col(smallestId);
    if (eigenvector.dot(surfaceNormalPositiveAxis_) < 0.0) eigenvector = -eigenvector;
    fusedMap_.at("surface_normal_x", *areaIterator) = eigenvector.x();
    fusedMap_.at("surface_normal_y", *areaIterator) = eigenvector.y();
    fusedMap_.at("surface_normal_z", *areaIterator) = eigenvector.z();
    timer.stop();
  }

  ROS_INFO("Surface normals have been computed in %f s.", Timing::getTotalSeconds(timerId));
  ROS_DEBUG("Mean: %f s, Min: %f s, Max: %f s.", Timing::getMeanSeconds(timerId), Timing::getMinSeconds(timerId), Timing::getMaxSeconds(timerId));
  Timing::reset(timerId);
  return true;
}


bool ElevationMap::clear()
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  rawMap_.clearAll();
  rawMap_.resetTimestamp();
  fusedMap_.clearAll();
  fusedMap_.resetTimestamp();
  return true;
}

void ElevationMap::move(const Eigen::Vector2d& position)
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  std::vector<BufferRegion> newRegions;

  if (rawMap_.move(position, newRegions)) {
    ROS_DEBUG("Elevation map has been moved to position (%f, %f).", rawMap_.getPosition().x(), rawMap_.getPosition().y());
  }

  if (hasUnderlyingMap_) {
    for (const auto& region : newRegions) {
      for (SubmapIterator iterator(rawMap_, region); !iterator.isPastEnd(); ++iterator) {
        Position position;
        rawMap_.getPosition(*iterator, position);
        rawMap_.at("elevation", *iterator) = underlyingMap_.atPosition("elevation", position);
      }
    }
  }
}

bool ElevationMap::publishRawElevationMap()
{
  if (!hasRawMapSubscribers()) return false;
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
  grid_map::GridMap rawMapCopy = rawMap_;
  scopedLock.unlock();
  rawMapCopy.add("standard_deviation", rawMapCopy.get("variance").array().sqrt().matrix());
  rawMapCopy.add("horizontal_standard_deviation", (rawMapCopy.get("horizontal_variance_x") + rawMapCopy.get("horizontal_variance_y")).array().sqrt().matrix());
  rawMapCopy.add("two_sigma_bound", rawMapCopy.get("elevation") + 2.0 * rawMapCopy.get("variance").array().sqrt().matrix());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(rawMapCopy, message);
  elevationMapRawPublisher_.publish(message);
  ROS_DEBUG("Elevation map raw has been published.");
  return true;
}

bool ElevationMap::publishFusedElevationMap()
{
  if (!hasFusedMapSubscribers()) return false;
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  GridMap fusedMapCopy = fusedMap_;
  scopedLock.unlock();
  fusedMapCopy.add("standard_deviation", fusedMapCopy.get("variance").array().sqrt().matrix());
  fusedMapCopy.add("two_sigma_bound", fusedMapCopy.get("elevation") + 2.0 * fusedMapCopy.get("variance").array().sqrt().matrix());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(fusedMapCopy, message);
  elevationMapFusedPublisher_.publish(message);
  ROS_DEBUG("Elevation map (fused) has been published.");
  return true;
}

grid_map::GridMap& ElevationMap::getRawGridMap()
{
  return rawMap_;
}

grid_map::GridMap& ElevationMap::getFusedGridMap()
{
  return fusedMap_;
}

ros::Time ElevationMap::getTimeOfLastUpdate()
{
  return ros::Time().fromNSec(rawMap_.getTimestamp());
}

ros::Time ElevationMap::getTimeOfLastFusion()
{
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  return ros::Time().fromNSec(fusedMap_.getTimestamp());
}

const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& ElevationMap::getPose()
{
  return pose_;
}

bool ElevationMap::getPosition3dInRobotParentFrame(const Eigen::Array2i& index, kindr::phys_quant::eigen_impl::Position3D& position)
{
  kindr::phys_quant::eigen_impl::Position3D positionInGridFrame;
  if (!rawMap_.getPosition3("elevation", index, positionInGridFrame.vector())) return false;
  position = pose_.transform(positionInGridFrame);
  return true;
}

boost::recursive_mutex& ElevationMap::getFusedDataMutex()
{
  return fusedMapMutex_;
}

boost::recursive_mutex& ElevationMap::getRawDataMutex()
{
  return rawMapMutex_;
}

bool ElevationMap::clean()
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  rawMap_.get("variance") = rawMap_.get("variance").unaryExpr(VarianceClampOperator<double>(minVariance_, maxVariance_));
  rawMap_.get("horizontal_variance_x") = rawMap_.get("horizontal_variance_x").unaryExpr(VarianceClampOperator<double>(minHorizontalVariance_, maxHorizontalVariance_));
  rawMap_.get("horizontal_variance_y") = rawMap_.get("horizontal_variance_y").unaryExpr(VarianceClampOperator<double>(minHorizontalVariance_, maxHorizontalVariance_));
  return true;
}

void ElevationMap::resetFusedData()
{
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  fusedMap_.clearAll();
  fusedMap_.resetTimestamp();
}

void ElevationMap::setFrameId(const std::string& frameId)
{
  rawMap_.setFrameId(frameId);
  fusedMap_.setFrameId(frameId);
}

const std::string& ElevationMap::getFrameId()
{
  return rawMap_.getFrameId();
}

bool ElevationMap::hasRawMapSubscribers() const
{
  if (elevationMapRawPublisher_.getNumSubscribers() < 1) return false;
  return true;
}

bool ElevationMap::hasFusedMapSubscribers() const
{
  if (elevationMapFusedPublisher_.getNumSubscribers() < 1) return false;
  return true;
}

void ElevationMap::underlyingMapCallback(const grid_map_msgs::GridMap& underlyingMap)
{
  ROS_INFO("Updating underlying map.");
  GridMapRosConverter::fromMessage(underlyingMap, underlyingMap_);
  if (underlyingMap_.getFrameId() != rawMap_.getFrameId()) {
    ROS_ERROR_STREAM("The underlying map does not have the same map frame (" <<underlyingMap_.getFrameId()
                     << ") as the elevation map (" << rawMap_.getFrameId() << ").");
    return;
  }
  hasUnderlyingMap_ = true;
}

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation)
{
  return 0.5 * erfc(-(x-mean)/(standardDeviation*sqrt(2.0)));
}

} /* namespace */
