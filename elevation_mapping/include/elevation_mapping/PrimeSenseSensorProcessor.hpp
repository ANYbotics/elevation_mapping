/*
 * PrimeSenseSensorProcessor.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: hannes
 */

#ifndef PRIME_SENSE_SENSOR_PROCESSOR_HPP_
#define PRIME_SENSE_SENSOR_PROCESSOR_HPP_

#include <elevation_mapping/SensorProcessor.hpp>

namespace elevation_mapping {

/*
 * Sensor processor for PrimeSense structured light sensors.
 * Cleans the point cloud, transforms it to a desired frame, and
 * computes the measurement variances based on a sensor model in
 * the desired frame.
 */
class PrimeSenseSensorProcessor : public SensorProcessor {
 public:
  PrimeSenseSensorProcessor(tf::TransformListener& transformListener);

  virtual ~PrimeSenseSensorProcessor();

 private:

  //! Points below the minimal and above the maximal sensor cutoff value are dropped.
  virtual bool cleanPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

  /*! PrimeSense sensor model:
   * standardDeviationInNormalDirection = sensorModelNormalFactorA_ + sensorModelNormalFactorB_ * (measurementDistance - sensorModelNormalFactorC_)^2;
   * standardDeviationInLateralDirection = sensorModelLateralFactor_ * measurementDistance
   * Taken from: Nguyen, C. V., Izadi, S., & Lovell, D., Modeling Kinect Sensor Noise for Improved 3D Reconstruction and Tracking, 2012.
   */
  virtual bool computeVariances(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
      const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
      Eigen::VectorXf& variances);
};

} /* namespace elevation_mapping */

#endif /* PRIME_SENSE_SENSOR_PROCESSOR_HPP_ */
