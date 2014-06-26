/*
 * KinectSensorProcessor.hpp
 *
 *  Created on: Feb 5, 2014
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

#include <elevation_mapping/sensor_processors/SensorProcessorBase.hpp>

namespace elevation_mapping {

/*!
 * Sensor processor for PrimeSense structured light sensors.
 */
class KinectSensorProcessor: public SensorProcessorBase
{
public:
  KinectSensorProcessor(tf::TransformListener& transformListener);

	virtual ~KinectSensorProcessor();

private:

	//! Points below the minimal and above the maximal sensor cutoff value are dropped.
	virtual bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

	virtual bool computeVariances(
			const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
			const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
			Eigen::VectorXf& variances);
};


} /* namespace elevation_mapping */
