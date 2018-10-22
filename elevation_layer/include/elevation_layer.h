//
// Created by eugenio on 15.10.18.
//

#ifndef ELEVATION_LAYER_H
#define ELEVATION_LAYER_H

#include <ros/ros.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/observation_buffer.h>
#include <message_filters/subscriber.h>
#include "grid_map_ros/GridMapRosConverter.hpp"
#include <filters/filter_chain.h>

#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/footprint.h>
#include <elevation_layer/ElevationPluginConfig.h>


namespace elevation_layer
{
class ElevationLayer : public costmap_2d::CostmapLayer
    {
    public:
        ElevationLayer()
        {
            costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
        }

        virtual ~ElevationLayer();
        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                  double* max_x, double* max_y);
        virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);

        virtual void activate();
        virtual void deactivate();
        virtual void reset();

        void elevationMapCallback(const grid_map_msgs::GridMapConstPtr& occupancy_grid);

    protected:
        std::string global_frame_;  ///< @brief The global frame for the costmap
        std::vector<boost::shared_ptr<message_filters::SubscriberBase> > elevation_subscribers_;  ///< @brief Used for the observation message filters
        dynamic_reconfigure::Server<elevation_layer::ElevationPluginConfig> *dsrv_;
        virtual void setupDynamicReconfigure(ros::NodeHandle& nh);


        int combination_method_;
        std::vector<geometry_msgs::Point> transformed_footprint_;
        bool rolling_window_;
        bool footprint_clearing_enabled_;
        bool elevation_map_received_;
        void updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                             double* max_x, double* max_y);

    private:
        void reconfigureCB(elevation_layer::ElevationPluginConfig &config, uint32_t level);
        grid_map::GridMap elevation_map_;
        ros::Subscriber elevation_subscriber_;
        double height_treshold_;
        std::string elevation_topic_;

        //! Filter chain.
        filters::FilterChain<grid_map::GridMap> filterChain_;

        //! Filter chain parameters name.
        std::string filter_chain_parameters_name_;

        bool filters_configuration_loaded_;
        std::string layer_name_;
        };

}   // namespace elevation_layer


#endif //ELEVATION_LAYER_H
