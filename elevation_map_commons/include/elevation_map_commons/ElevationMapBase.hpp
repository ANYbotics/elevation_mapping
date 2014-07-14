/*
 * ElevationMapBase.hpp
 *
 *  Created on: Jul 14, 2014
 *      Author: Péter Fankhauser
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// STL
#include <map>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// Kindr
#include <kindr/poses/PoseEigen.hpp>
#include <kindr/phys_quant/PhysicalQuantitiesEigen.hpp>

// ROS
#include <ros/time.h>

namespace elevation_map_commons {

/*!
 * Elevation map base class.
 * When deriving this class, you have to initialize the data structure like this
 *
 *      mapFloatData_.insert(std::pair<FloatDataType, MatrixXf>(FloatDataType::Elevation, MatrixXf()));
 *
 * in the constructor for all the data types that should be stored in the class.
 * The elevation data type is added by default.
 */

class ElevationMapBase
{
 public:

  /*!
   * Constructor.
   */
  ElevationMapBase();

  /*!
   * Destructor.
   */
  virtual ~ElevationMapBase();

  /*!
   * Set the geometry of the elevation map. Resets all the data.
   * @param length the side lengths in x, and y-direction of the elevation map [m].
   * @param position the position of the elevation map in the elevation map frame [m].
   * @param resolution the cell size in [m/cell].
   * @return true if successful.
   */
  virtual bool setGeometry(const Eigen::Array2d& length, const kindr::phys_quant::eigen_impl::Position3D& position,
                           const double& resolution);

  /*!
   * Set the id and pose of the elevation map frame.
   * @param frameId the id of the elevation map frame.
   * @param pose the pose of the elevation map frame w.r.t. the parent frame.
   */
  void setFrame(const std::string& frameId,
                const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& pose);

  /*!
   * Gets a submap from the map. The requested submap is specified with the requested location (usually the center)
   * and length. The returned position of the submap might be different as the submap might not have the same
   * length as the requested length due to the borders of the map.
   * @param[in] position the requested position of the submap.
   * @param[in] length the requested length of the submap.
   * @param[out] indexInSubmap the index of the requested position in the submap.
   * @param[out] success true if successful, false otherwise.
   * @return submap (is empty if success is false).
   */
  ElevationMapBase getSubmap(const Eigen::Vector2d& position, const Eigen::Array2d& length,
                             Eigen::Array2i& indexInSubmap, bool& success);

  /*!
   * Relocate the elevation map frame w.r.t. the parent frame. Use this to move
   * the elevation map boundaries without moving the elevation map data. Takes care
   * of all the data handling, such that the elevation map is stationary in the parent frame.
   * @param mapFramePosition the new location of the elevation map frame in the parent frame.
   * @return true if successful.
   */
  bool relocate(const kindr::phys_quant::eigen_impl::Position3D& mapFramePosition);

  /*!
   * Reset data structures of the elevation map that make cell invalid.
   * Header information (geometry etc.) remains valid.
   */
  void reset();

  /*!
   * Get the time stamp of the map data.
   * @return time stamp.
   */
  const ros::Time& getTimeStamp();

  /*!
   * Set time stamp of the map data.
   * @param timeStamp the time stamp to set.
   */
  void setTimeStamp(ros::Time& timeStamp);

  /*!
   * Get the side length of the map.
   * @return side length of the map.
   */
  const Eigen::Array2d& getLength();

  /*!
   * Get the position of the map in the elevation map frame.
   * @return position of the map in the elevation map frame.
   */
  const kindr::phys_quant::eigen_impl::Position3D& getPosition();

  /*!
   * Get the resolution of the elevation map.
   * @return resolution of the elevation map in the xy plane [m/cell].
   */
  double getResolution();

  /*!
   * Get the pose of the elevation map frame w.r.t. the parent frame.
   * @return pose of the elevation map frame w.r.t. the parent frame.
   */
  const kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD& getPose();

  /*!
   * Get the buffer size (rows and cols of the data structure).
   * @return the buffer size.
   */
  const Eigen::Array2i& getBufferSize();

  /*!
   * Get the start index of the circular buffer.
   * @return the buffert start index.
   */
  const Eigen::Array2i& getBufferStartIndex();

  /*!
   * Definition of the types for float-typed data.
   */
  enum class FloatDataType
  {
    Elevation,            //!< Height data.
    Variance,             //!< Variance of the height.
    HorizontalVarianceX,  //!< Horizontal position variance in x-direction.
    HorizontalVarianceY   //!< Horizontal position variance in y-direction.
  };

  /*!
   * Definition of the types for int-typed data.
   */
  enum class IntDataType
  {
    Color                 //!< Color data.
  };

 protected:

  /*!
   * Reset all the data structures of the elevation map (less efficient than reset()).
   * Header information (geometry etc.) remains valid.
   */
  void resetAll();

  /*!
   * Resets a number of columns in the raw elevation map.
   * @param index the left index for the columns to be reset.
   * @param nCols the number of columns to reset.
   */
  void resetCols(unsigned int index, unsigned int nCols);

  /*!
   * Resets a number of rows in the raw elevation map.
   * @param index the upper index for the rows to be reset.
   * @param nRows the number of rows to reset.
   */
  void resetRows(unsigned int index, unsigned int nRows);

  /*!
   * Resize the buffer.
   * @param bufferSize the requested buffer size.
   */
  void resizeBuffer(const Eigen::Array2i& bufferSize);

  /*
   * Header.
   */

  //! Frame id of the elevation map.
  std::string frameId_;

  //! Time stamp of the map data.
  ros::Time timeStamp_;

  //! Pose of the elevation map frame w.r.t. the parent frame.
  kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD pose_;

  //! Map position in the elevation map frame [m].
  kindr::phys_quant::eigen_impl::Position3D position_;

  //! Side length of the map in x- and y-direction [m].
  Eigen::Array2d length_;

  //! Map resolution in xy plane [m/cell].
  double resolution_;

  /*
   * Data storage (buffer).
   */

  //! Size of the buffer (rows and cols of the data structure).
  Eigen::Array2i bufferSize_;

  //! Circular buffer start indeces.
  Eigen::Array2i bufferStartIndex_;

  //! Map data as matrix.
  std::map<FloatDataType, Eigen::MatrixXf> mapFloatData_;
  std::map<IntDataType, Eigen::Matrix<unsigned long, Eigen::Dynamic, Eigen::Dynamic>> mapIntData_;

  //! Definitions of the reset values for the cells of the different data types.
  std::map<FloatDataType, const float> mapFloatResetValues_;
  std::map<IntDataType, const unsigned long> mapIntResetValues_;
};

} /* namespace elevation_map_commons */
