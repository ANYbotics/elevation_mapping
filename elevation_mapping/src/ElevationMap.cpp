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
#include <grid_map_lib/GridMapMath.hpp>
#include <grid_map/GridMapMsgHelpers.hpp>
#include <grid_map_lib/SubmapIterator.hpp>

// Math
#include <math.h>

// ROS Logging
#include <ros/ros.h>

// Eigenvalues
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;
using namespace sm;
using namespace sm::timing;

namespace elevation_mapping {

ElevationMap::ElevationMap(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle),
      rawMap_(vector<string>({"elevation", "variance", "horizontal_variance_x", "horizontal_variance_y", "color"})),
      fusedMap_(vector<string>({"elevation", "variance", "color", "surface_normal_x", "surface_normal_y", "surface_normal_z"}))
{
  minVariance_ = 0.0;
  maxVariance_ = 0.0;
  mahalanobisDistanceThreshold_ = 0.0;
  multiHeightNoise_ = 0.0;
  minHorizontalVariance_ = 0.0;
  maxHorizontalVariance_ = 0.0;
  rawMap_.setClearTypes(vector<string>({"elevation", "variance"}));
  fusedMap_.setClearTypes(vector<string>({"elevation", "variance"}));
  clear();

  elevationMapRawPublisher_ = nodeHandle_.advertise<grid_map_msg::GridMap>("elevation_map_raw", 1);
  elevationMapFusedPublisher_ = nodeHandle_.advertise<grid_map_msg::GridMap>("elevation_map", 1);
}

ElevationMap::~ElevationMap()
{

}

void ElevationMap::setGeometry(const Eigen::Array2d& length, const double& resolution, const Eigen::Vector2d& position)
{
  boost::recursive_mutex::scoped_lock scopedLockForRawData(rawMapMutex_);
  boost::recursive_mutex::scoped_lock scopedLockForFusedData(fusedMapMutex_);
  rawMap_.setGeometry(length, resolution, position);
  fusedMap_.setGeometry(length, resolution, position);
  ROS_INFO_STREAM("Elevation map grid resized to " << rawMap_.getBufferSize()(0) << " rows and "  << rawMap_.getBufferSize()(1) << " columns.");
}

bool ElevationMap::add(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud, Eigen::VectorXf& pointCloudVariances)
{
  for (unsigned int i = 0; i < pointCloud->size(); ++i)
  {
    auto& point = pointCloud->points[i];

    Array2i index;
    Vector2d position(point.x, point.y);
    if (!rawMap_.getIndex(position, index)) continue; // Skip this point if it does not lie within the elevation map.

    auto& elevation = rawMap_.at("elevation", index);
    auto& variance = rawMap_.at("variance", index);
    auto& horizontalVarianceX = rawMap_.at("horizontal_variance_x", index);
    auto& horizontalVarianceY = rawMap_.at("horizontal_variance_y", index);
    auto& color = rawMap_.at("color", index);
    float pointVariance = pointCloudVariances(i);

    if (!rawMap_.isValid(index))
    {
      // No prior information in elevation map, use measurement.
      elevation = point.z;
      variance = pointVariance;
      horizontalVarianceX = minHorizontalVariance_;
      horizontalVarianceY = minHorizontalVariance_;
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);
      continue;
    }

    double mahalanobisDistance = sqrt(pow(point.z - elevation, 2) / variance);

    if (mahalanobisDistance < mahalanobisDistanceThreshold_)
    {
      // Fuse measurement with elevation map data.
      elevation = (variance * point.z + pointVariance * elevation) / (variance + pointVariance);
      variance =  (pointVariance * variance) / (pointVariance + variance);
      // TODO Add color fusion.
      grid_map::colorVectorToValue(point.getRGBVector3i(), color);
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

  const auto& bufferSize = rawMap_.getBufferSize();

  if (!(
      (Array2i(varianceUpdate.rows(), varianceUpdate.cols()) == bufferSize).all() &&
      (Array2i(horizontalVarianceUpdateX.rows(), horizontalVarianceUpdateX.cols()) == bufferSize).all() &&
      (Array2i(horizontalVarianceUpdateY.rows(), horizontalVarianceUpdateY.cols()) == bufferSize).all()
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
  return fuse(Array2i(0, 0), fusedMap_.getBufferSize(), computeSurfaceNormals);
}

bool ElevationMap::fuseArea(const Eigen::Vector2d& position, const Eigen::Array2d& length, const bool computeSurfaceNormals)
{
  ROS_DEBUG("Requested to fuse an area of the elevation map with center at (%f, %f) and side lenghts (%f, %f)",
            position[0], position[1], length[0], length[1]);

  Array2i topLeftIndex;
  Array2i submapBufferSize;

  // These parameters are not used in this function.
  Vector2d submapPosition;
  Array2d submapLength;
  Array2i requestedIndexInSubmap;

  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);

  grid_map_lib::getSubmapInformation(topLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap, position, length,
                                     rawMap_.getLength(), rawMap_.getPosition(), rawMap_.getResolution(), rawMap_.getBufferSize(),
                                     rawMap_.getBufferStartIndex());

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

  ROS_DEBUG("Fusing elevation map2...");

  // Check if there is the need to reset out-dated data.
  if (fusedMap_.getTimestamp() != rawMapCopy.getTimestamp()) resetFusedData();

  // Align fused map with raw map.
  if (rawMapCopy.getPosition() != fusedMap_.getPosition()) fusedMap_.move(rawMapCopy.getPosition());

  // For each cell in requested area.
  for (grid_map_lib::SubmapIterator areaIterator(rawMapCopy, topLeftIndex, size); !areaIterator.isPassedEnd(); ++areaIterator) {
    if (timer.isTiming()) timer.stop();
    timer.start();

    // Check if fusion for this cell has already been done earlier.
    if (fusedMap_.isValid(*areaIterator)) continue;

    if (!rawMapCopy.isValid(*areaIterator)) {
      // This is an empty cell (hole in the map).
      continue;
    }

    // Size of submap (2 sigma bound). TODO Add minimum/maximum submap size?
    Array2d requestedSubmapLength = 4.0 * Array2d(rawMapCopy.at("horizontal_variance_x", *areaIterator), rawMapCopy.at("horizontal_variance_y", *areaIterator)).sqrt();

    // Requested position (center) of submap in map.
    Vector2d requestedSubmapPosition;
    rawMapCopy.getPosition(*areaIterator, requestedSubmapPosition);

    Array2d submapLength;
    Vector2d submapPosition;
    Array2i submapTopLeftIndex, submapBufferSize, requestedIndexInSubmap;

    grid_map_lib::getSubmapInformation(submapTopLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap, requestedSubmapPosition, requestedSubmapLength,
                         rawMapCopy.getLength(), rawMapCopy.getPosition(), rawMapCopy.getResolution(), rawMapCopy.getBufferSize(), rawMapCopy.getBufferStartIndex());

//    cout << "submapTopLeftIndex: " << submapTopLeftIndex << endl;
//    cout << "submapBufferSize: " << submapBufferSize << endl;
//    cout << "submapPosition: " << submapPosition << endl;
//    cout << "submapLength: " << submapLength << endl;
//    cout << "requestedIndexInSubmap: " << requestedIndexInSubmap << endl;
//    cout << "requestedSubmapPosition: " << requestedSubmapPosition << endl;
//    cout << "requestedSubmapLength: " << requestedSubmapLength << endl;
//    cout << "rawMapCopy.getLength(): " << rawMapCopy.getLength() << endl;
//    cout << "rawMapCopy.getPosition(): " << rawMapCopy.getPosition() << endl;
//    cout << "rawMapCopy.getResolution(): " << rawMapCopy.getResolution() << endl;
//    cout << "rawMapCopy.getBufferSize(): " << rawMapCopy.getBufferSize() << endl;
//    cout << "rawMapCopy.getBufferStartIndex(): " << rawMapCopy.getBufferStartIndex() << endl;

    // Prepare data fusion.
    ArrayXf means, variances, weights;
    int maxNumberOfCellsToFuse = submapBufferSize.prod();
    means.resize(maxNumberOfCellsToFuse);
    variances.resize(maxNumberOfCellsToFuse);
    weights.resize(maxNumberOfCellsToFuse);

    // For each cell in submap.
    size_t i = 0;
    for (grid_map_lib::SubmapIterator submapIterator(rawMapCopy, submapTopLeftIndex, submapBufferSize); !submapIterator.isPassedEnd(); ++submapIterator) {
      if (!rawMapCopy.isValid(*submapIterator)) {
        // Empty cell in submap (cannot be center cell because we checked above).
        continue;
      }

      means[i] = rawMapCopy.at("elevation", *submapIterator);
      variances[i] = rawMapCopy.at("variance", *submapIterator);

      // Compute weight from probability.
      Vector2d position;
      rawMapCopy.getPosition(*submapIterator, position);

      Vector2d distanceToCenter = (position - submapPosition).cwiseAbs();

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

  ROS_DEBUG("Fusing elevation map...");

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
  for (grid_map_lib::SubmapIterator areaIterator(fusedMap_, topLeftIndex, size); !areaIterator.isPassedEnd(); ++areaIterator) {
    if (timer.isTiming()) timer.stop();
    timer.start();

    // Check if this is an empty cell (hole in the map).
    if (!fusedMap_.isValid(*areaIterator)) continue;
    // Check if surface normal for this cell has already been computed earlier.
    if (fusedMap_.isValid(*areaIterator, surfaceNormalTypes)) continue;

    // Size of submap area for surface normal estimation.
    Array2d submapLength = Array2d::Ones() * (2.0 * surfaceNormalEstimationRadius_);

    // Requested position (center) of submap in map.
    Vector2d submapPosition;
    fusedMap_.getPosition(*areaIterator, submapPosition);
    Array2i submapTopLeftIndex, submapBufferSize, requestedIndexInSubmap;
    grid_map_lib::getSubmapInformation(submapTopLeftIndex, submapBufferSize, submapPosition, submapLength, requestedIndexInSubmap, submapPosition, submapLength,
                                       fusedMap_.getLength(), fusedMap_.getPosition(), fusedMap_.getResolution(), fusedMap_.getBufferSize(), fusedMap_.getBufferStartIndex());

    // Prepare data computation.
    const int maxNumberOfCells = submapBufferSize.prod();
    MatrixXd points(3, maxNumberOfCells);

    // Gather sourounding data.
    size_t nPoints = 0;
    for (grid_map_lib::SubmapIterator submapIterator(fusedMap_, submapTopLeftIndex, submapBufferSize); !submapIterator.isPassedEnd(); ++submapIterator) {
      if (!fusedMap_.isValid(*submapIterator)) continue;
      Vector3d point;
      fusedMap_.getPosition3d("elevation", *submapIterator, point);
      points.col(nPoints) = point;
      nPoints++;
    }
//    nPoints.conservativeResize(3, nPoints); // TODO Eigen version?

    // Compute eigenvectors.
    const Vector3d mean = points.leftCols(nPoints).rowwise().sum() / nPoints;
    const MatrixXd NN = points.leftCols(nPoints).colwise() - mean;

    const Matrix3d covarianceMatrix(NN * NN.transpose());
    Vector3d eigenvalues = Vector3d::Identity();
    Matrix3d eigenvectors = Matrix3d::Identity();
    // Ensure that the matrix is suited for eigenvalues calculation
    if (covarianceMatrix.fullPivHouseholderQr().rank() >= 3) {
      const EigenSolver<MatrixXd> solver(covarianceMatrix);
      eigenvalues = solver.eigenvalues().real();
      eigenvectors = solver.eigenvectors().real();
    } else {
      ROS_DEBUG("Covariance matrix needed for eigen decomposition is degenerated. Expected cause: no noise in data (nPoints = %i)", nPoints);
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
    Vector3d eigenvector = eigenvectors.col(smallestId);
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
  rawMap_.move(position);
}

bool ElevationMap::publishRawElevationMap()
{
  if (elevationMapRawPublisher_.getNumSubscribers() < 1) return false;
  boost::recursive_mutex::scoped_lock scopedLock(rawMapMutex_);
  grid_map::GridMap rawMapCopy = rawMap_;
  scopedLock.unlock();
  rawMapCopy.add("standard_deviation", rawMapCopy.get("variance").array().sqrt().matrix());
  rawMapCopy.add("horizontal_standard_deviation", (rawMapCopy.get("horizontal_variance_x") + rawMapCopy.get("horizontal_variance_y")).array().sqrt().matrix());
  rawMapCopy.add("two_sigma_bound", rawMapCopy.get("elevation") + 2.0 * rawMapCopy.get("variance").array().sqrt().matrix());
  grid_map_msg::GridMap message;
  rawMapCopy.toMessage(message);
  elevationMapRawPublisher_.publish(message);
  ROS_DEBUG("Elevation map raw has been published.");
  return true;
}

bool ElevationMap::publishElevationMap()
{
  if (elevationMapFusedPublisher_.getNumSubscribers() < 1) return false;
  boost::recursive_mutex::scoped_lock scopedLock(fusedMapMutex_);
  grid_map::GridMap fusedMapCopy = fusedMap_;
  scopedLock.unlock();
  fusedMapCopy.add("standard_deviation", fusedMapCopy.get("variance").array().sqrt().matrix());
  fusedMapCopy.add("two_sigma_bound", fusedMapCopy.get("elevation") + 2.0 * fusedMapCopy.get("variance").array().sqrt().matrix());
  grid_map_msg::GridMap message;
  fusedMapCopy.toMessage(message);
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
  if (!rawMap_.getPosition3d("elevation", index, positionInGridFrame.vector())) return false;
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

float ElevationMap::cumulativeDistributionFunction(float x, float mean, float standardDeviation)
{
  return 0.5 * erfc(-(x-mean)/(standardDeviation*sqrt(2.0)));
}

} /* namespace */
