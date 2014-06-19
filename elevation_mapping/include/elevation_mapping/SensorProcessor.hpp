/*
 * SensorProcessor.hpp
 *
 *  Created on: Jun 6, 2014
 *      Author: hannes
 */

#ifndef ELEVATION_MAPPING_SENSOR_PROCESSOR_HPP_
#define ELEVATION_MAPPING_SENSOR_PROCESSOR_HPP_

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// Eigen
#include <Eigen/Core>

// kindr
#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

//STL
#include <vector>
#include <string>

namespace elevation_mapping {

/*!
 * Generic Sensor processor base class. Provides functionalities
 * common to all sensors and defines interface for specialized
 * sensor processor classes.
 * Cleans the point cloud, transforms it to a desired frame, and
 * computes the measurement variances based on a sensor model in
 * the desired frame.
 */
class SensorProcessor {
 public:
  typedef boost::shared_ptr<SensorProcessor> Ptr;
  typedef const boost::shared_ptr<SensorProcessor> ConstPtr;

  SensorProcessor(tf::TransformListener& transformListener);

  virtual ~SensorProcessor();

  /*!
   * Processes the point cloud.
   * @param[in] pointCloudInput the input point cloud.
   * @param[in] targetFrame the frame to which the point cloud should be transformed.
   * @param[out] pointCloudOutput the processed point cloud.
   * @param[out] variances the measurement variances expressed in the target frame.
   * @return true if successful.
   */
  bool process(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloudInput,
      const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudOutput,
      Eigen::VectorXf& variances);

  //Frame ID accessors
  void setBaseFrameId(std::string baseFrameId);
  void setMapFrameId(std::string mapFrameId);

  std::string getBaseFrameId() const;
  std::string getMapFrameId() const;

  //Parameter accessors
  double getSensorParameter(std::size_t index) const;
  std::string getSensorParameterName(std::size_t index) const;

  void setTransformListenerTimeout(ros::Duration timeout);
  ros::Duration getTransformListenerTimeout() const;

  friend class ElevationMapping;

 protected:
  virtual bool cleanPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud) = 0;

  virtual bool computeVariances(
      const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pointCloud,
      const Eigen::Matrix<double, 6, 6>& robotPoseCovariance,
      Eigen::VectorXf& variances) = 0;

  bool updateTransformations(std::string sensorFrameId, ros::Time timeStamp);

  bool transformPointCloud(
      const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloud,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudTransformed,
      const std::string& targetFrame);

  //! TF transform listener.
  tf::TransformListener& transformListener_;

  //! The timeout duration for the lookup of the transformation between sensor frame and target frame.
  ros::Duration transformListenerTimeout_;

  //! Rotation from Base to Sensor frame (C_SB)
  kindr::rotations::eigen_impl::RotationMatrixPD rotationBaseToSensor_;

  //! Translation from Base to Sensor in Base frame (B_r_BS)
  kindr::phys_quant::eigen_impl::Position3D translationBaseToSensorInBaseFrame_;

  //! Rotation from (elevation) Map to Base frame (C_BM)
  kindr::rotations::eigen_impl::RotationMatrixPD rotationMapToBase_;

  //! Translation from Map to Base in Map frame (M_r_MB)
  kindr::phys_quant::eigen_impl::Position3D translationMapToBaseInMapFrame_;

  //! Transformation from Sensor to Map frame
  Eigen::Affine3d transformationSensorToMap_;

  //! TF frame id of the map.
  std::string mapFrameId_;

  //! TF frame id of the base.
  std::string baseFrameId_;

  //! Sensor parameters. Initialized by ElevationMapping friend class.
  std::vector<double> sensorParameters_;

  //! Sensor parameter names. Must be initialized by derived sensor processor class.
  std::vector<std::string> sensorParameterNames_;

};

} /* namespace elevation_mapping */

#endif /* ELEVATION_MAPPING_SENSOR_PROCESSOR_HPP_ */
