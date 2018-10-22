//
// Created by eugenio on 16.10.18.
//

#include <elevation_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(elevation_layer::ElevationLayer, costmap_2d::Layer)

using costmap_2d::NO_INFORMATION;
using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::FREE_SPACE;

using costmap_2d::ObservationBuffer;
using costmap_2d::Observation;

namespace elevation_layer
{

    void ElevationLayer::onInitialize() {
        ros::NodeHandle nh("~/" + name_), g_nh;
        rolling_window_ = layered_costmap_->isRolling();

        bool track_unknown_space;
        nh.param("track_unknown_space", track_unknown_space, layered_costmap_->isTrackingUnknown());
        if (track_unknown_space)
            default_value_ = NO_INFORMATION;
        else
            default_value_ = FREE_SPACE;

        ElevationLayer::matchSize();
        current_ = true;
        elevation_map_received_ = false;
        global_frame_ = layered_costmap_->getGlobalFrameID();

        // get our tf prefix
        ros::NodeHandle prefix_nh;
        const std::string tf_prefix = tf::getPrefixParam(prefix_nh);

        // get the topics that we'll subscribe to from the parameter server
        nh.param("elevation_topic", elevation_topic_, std::string(""));
        ROS_INFO("    Subscribed to Topics: %s", elevation_topic_.c_str());
        nh.param("height_treshold", height_treshold_, 0.1);
        ROS_INFO_STREAM("height_treshold" << height_treshold_);

        elevation_subscriber_ = nh.subscribe(elevation_topic_, 1, &ElevationLayer::elevationMapCallback, this);
        dsrv_ = NULL;
        setupDynamicReconfigure(nh);
    }

    ElevationLayer::~ElevationLayer()
    {
        if (dsrv_)
            delete dsrv_;
    }

    void ElevationLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y)
    {
        if (rolling_window_)
            updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
        if (!enabled_ || !elevation_map_received_)
            return;
        useExtraBounds(min_x, min_y, max_x, max_y);


        for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index gridmap_index(*iterator);
            grid_map::Position vertexPositionXY;
            elevation_map_.getPosition(gridmap_index, vertexPositionXY);
            double px = vertexPositionXY.x();
            double py = vertexPositionXY.y();

            touch(px, py, min_x, min_y, max_x, max_y);
        }
        updateFootprint(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
    }

    void ElevationLayer::updateFootprint(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y,
                                        double* max_x, double* max_y)
    {
        if (!footprint_clearing_enabled_) return;
        costmap_2d::transformFootprint(robot_x, robot_y, robot_yaw, getFootprint(), transformed_footprint_);

        for (unsigned int i = 0; i < transformed_footprint_.size(); i++)
        {
            touch(transformed_footprint_[i].x, transformed_footprint_[i].y, min_x, min_y, max_x, max_y);
        }
    }

    void ElevationLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_ || !elevation_map_received_)
            return;


        grid_map::Matrix& data = elevation_map_["elevation"];   //TODO: put layer name in config file
        for (grid_map::GridMapIterator iterator(elevation_map_); !iterator.isPastEnd(); ++iterator) {
            const grid_map::Index gridmap_index(*iterator);
            grid_map::Position vertexPositionXY;
            elevation_map_.getPosition(gridmap_index, vertexPositionXY);
            double px = vertexPositionXY.x();
            double py = vertexPositionXY.y();
            // now we need to compute the map coordinates for the observation
            unsigned int mx, my;
            if (!worldToMap(px, py, mx, my)) {
                ROS_WARN("Computing map coords failed");
                continue;
            }
            if ( data(gridmap_index(0), gridmap_index(1)) > height_treshold_ )  // If point too high, label as obstacle
            {
                master_grid.setCost(mx, my, LETHAL_OBSTACLE);
            }
        }

        if (footprint_clearing_enabled_)
        {
            setConvexPolygonCost(transformed_footprint_, costmap_2d::FREE_SPACE);
        }

        switch (combination_method_)
        {
            case 0:  // Overwrite
                updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);
                break;
            case 1:  // Maximum
                updateWithMax(master_grid, min_i, min_j, max_i, max_j);
                break;
            default:  // Nothing
                break;
        }
    }

    void ElevationLayer::elevationMapCallback(const grid_map_msgs::GridMapConstPtr& elevation)
    {
        if(!grid_map::GridMapRosConverter::fromMessage(*elevation, elevation_map_))
        {
            ROS_WARN("Grid Map msg Conversion failed !");
        }
        if(!elevation_map_received_)
        {
            elevation_map_received_ = true;
            ROS_INFO("map received !!!!!!!!!!!!!!!!!!!!!!!!!!!!!*****************");
        }
    }

    void ElevationLayer::setupDynamicReconfigure(ros::NodeHandle& nh)
    {
        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::ElevationPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::ElevationPluginConfig>::CallbackType cb = boost::bind(
                &ElevationLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
    }

    void ElevationLayer::reconfigureCB(costmap_2d::ElevationPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    void ElevationLayer::reset()
    {
        deactivate();
        resetMaps();
        current_ = true;
        activate();
    }

    void ElevationLayer::activate()
    {
        // if we're stopped we need to re-subscribe to topics
        ros::NodeHandle nh("~/" + name_);
        elevation_subscriber_ = nh.subscribe(elevation_topic_, 1, &ElevationLayer::elevationMapCallback, this);

    }
    void ElevationLayer::deactivate()
    {
        elevation_subscriber_.shutdown();
    }
}