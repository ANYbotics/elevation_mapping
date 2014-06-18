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

class PrimeSenseSensorProcessor: public SensorProcessor
{
public:
	PrimeSenseSensorProcessor(tf::TransformListener& transformListener);

	virtual ~PrimeSenseSensorProcessor();

private:

	virtual bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

	virtual bool computeVariances(
			const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
			const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
			Eigen::VectorXf& variances);
};


} /* namespace elevation_mapping */

#endif /* PRIME_SENSE_SENSOR_PROCESSOR_HPP_ */
