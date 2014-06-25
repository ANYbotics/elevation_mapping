/*
 * AslamSensorProcessor.hpp
 *
 *  Created on: Jun 18, 2014
 *      Author: hannes
 */

#ifndef ASLAM_SENSOR_PROCESSOR_HPP_
#define ASLAM_SENSOR_PROCESSOR_HPP_

#include <elevation_mapping/SensorProcessor.hpp>

namespace elevation_mapping {

/*
 * Sensor processor for the ASLAM stereo sensor.
 * Cleans the point cloud, transforms it to a desired frame, and
 * computes the measurement variances based on a sensor model in
 * the desired frame.
 */
class AslamSensorProcessor: public SensorProcessor
{
public:
	AslamSensorProcessor(tf::TransformListener& transformListener);

	virtual ~AslamSensorProcessor();

private:

	virtual bool cleanPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud);

	virtual bool computeVariances(
			const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
			const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
			Eigen::VectorXf& variances);
};


} /* namespace elevation_mapping */

#endif /* ASLAM_SENSOR_PROCESSOR_HPP_ */
