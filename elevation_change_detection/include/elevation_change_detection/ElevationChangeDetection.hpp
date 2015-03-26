/*
 * ElevationChangeDetection.hpp
 *
 *  Created on: Mar 26, 2015
 *      Author: Martin Wermelinger
 *	 Institute: ETH Zurich, Autonomous Systems Lab
 */

#pragma once

// Grid Map
#include <grid_map/GridMap.hpp>

// ROS
#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <string>

namespace elevation_change_detection {

  class ElevationChangeDetection {

  public:
    /*!
    * Constructor.
    */
    ElevationChangeDetection(ros::NodeHandle& nodeHandle);

    /*!
    * Destructor.
    */
    virtual ~ElevationChangeDetection();

    /*!
     * Publishes the (latest) elevation change map.
     * @return true if successful.
     */
    bool publishElevationChangeMap(const grid_map::GridMap& map);

  private:

    /*!
    * Reads and verifies the ROS parameters.
    * @return true if successful.
    */
    bool readParameters();

    /*!
    * Loads the ground truth elevation map from a ROS bag file.
    * @param[in] pathToBag string with the path to the Bag file containing the ground truth data
    * @return true if successful.
    */
    bool loadElevationMap(const std::string& pathToBag);

    /*!
    * Callback function for the update timer. Forces an update of the elevation
    * change map from a new elevation map requested from the grid map service.
    * @param timerEvent the timer event.
    */
    void updateTimerCallback(const ros::TimerEvent& timerEvent);

    /*!
     * Gets the grid map for the desired submap center point.
     * @param[out] map the map that is received.
     * @return true if successful, false if ROS service call failed.
     */
    bool getGridMap(grid_map_msg::GridMap& map);

    /*!
     * Get a submap of the ground truth map corresponding to the elevation map.
     * @param[in] requestedSubmapPosition the submap of the ground truth map.
     * @param[in] requestedSubmapPubmapLength the submap of the ground truth map.
     * @param[out] map the submap of the ground truth map.
     */
    void getGroundTruthSubmap(const Eigen::Vector2d& requestedSubmapPosition, const Eigen::Array2d& requestedSubmapPubmapLength, grid_map::GridMap& map);

    //! ROS node handle.
    ros::NodeHandle& nodeHandle_;

    //! Elevation map service client.
    ros::ServiceClient submapClient_;

    //! Name of the elevation submap service.
    std::string submapServiceName_;

    //! Timer for the map update.
    ros::Timer updateTimer_;

    //! Duration between map updates.
    ros::Duration updateDuration_;

    //! Center point of the requested map.
    geometry_msgs::PointStamped submapPoint_;

    //! Id of the frame of the elevation map.
    std::string mapFrameId_;

    //! TF listener.
    tf::TransformListener transformListener_;

    //! Requested map cell types.
    std::vector<std::string> requestedMapTypes_;

    //! Path to the ROS Bag containing the ground truth elevation map.
    std::string pathToBag_;

    //! Requested map length in [m].
    Eigen::Array2d mapLength_;

    //! Elevation map type
    const std::string type_;

    //! Publisher of the traversability occupancy grid.
    ros::Publisher elevationChangePublisher_;

    //! Ground Truth elevation map
    grid_map::GridMap groundTruthMap_;

  };

} /* namespace elevation_change_detection*/
