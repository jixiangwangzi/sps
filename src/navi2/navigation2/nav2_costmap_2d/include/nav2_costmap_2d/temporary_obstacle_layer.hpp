#ifndef NAV2_COSTMAP_2D__TEMPORARY_OBSTACLE_LAYER2_HPP_
#define NAV2_COSTMAP_2D__TEMPORARY_OBSTACLE_LAYER2_HPP_

#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "laser_geometry/laser_geometry.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wreorder"
#include "tf2_ros/message_filter.h"
#pragma GCC diagnostic pop
#include "message_filters/subscriber.h"
#include "message_filters/subscriber.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_costmap_2d/observation_buffer.hpp"
#include "nav2_costmap_2d/footprint.hpp"


namespace nav2_costmap_2d
{

class TemporaryObstacleLayer : public CostmapLayer
{
public:
    TemporaryObstacleLayer()
    {
        costmap_ = NULL;  // this is the unsigned char* member of parent class Costmap2D.
    }

    virtual ~TemporaryObstacleLayer();
    virtual void onInitialize();
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw,
        double* min_x, double* min_y,
        double* max_x, double* max_y);
    virtual void updateCosts(
        nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j,
        int max_i, int max_j);

    virtual void activate();
    virtual void deactivate();
    virtual void reset();

    virtual bool isClearable() {return true;}
    void resetBuffersLastUpdated();


    /**
     * @brief  A callback to handle buffering LaserScan messages
     * @param message The message returned from a message notifier
     * @param buffer A pointer to the observation buffer to update
     */
    void laserScanCallback(
        sensor_msgs::msg::LaserScan::ConstSharedPtr message,
        const std::shared_ptr<nav2_costmap_2d::ObservationBuffer>& buffer);

    /**
     * @brief A callback to handle buffering LaserScan messages which need filtering to turn Inf values into range_max.
     * @param message The message returned from a message notifier
     * @param buffer A pointer to the observation buffer to update
     */
    void laserScanValidInfCallback(
        sensor_msgs::msg::LaserScan::ConstSharedPtr message,
        const std::shared_ptr<nav2_costmap_2d::ObservationBuffer>& buffer);

    /**
     * @brief  A callback to handle buffering PointCloud2 messages
     * @param message The message returned from a message notifier
     * @param buffer A pointer to the observation buffer to update
     */
    void pointCloud2Callback(
        sensor_msgs::msg::PointCloud2::ConstSharedPtr message,
        const std::shared_ptr<nav2_costmap_2d::ObservationBuffer>& buffer);

protected:
    void updateFootprint(
        double robot_x, double robot_y, double robot_yaw);
    // virtual void setupDynamicReconfigure(ros::NodeHandle& nh);
    // void reconfigureCB(wr_costmap_2d::WRObstaclePluginConfig& config, uint32_t level);

private:
    /**
     * @brief Navigator task enabling/disabling callback
     * @param request_header Service request header
     * @param request Service request
     * @param response Service response
     */
    void enableCallback(
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response>);
    /**
     * @brief Clear this layer
    */    
    void pause();
    /**
     * @brief Start this layer
    */ 
    void start();
    
private: 
      // Clock
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};   
    std::string global_frame_;  ///< @brief The global frame for the costmap
    laser_geometry::LaserProjection projector_;  ///< @brief Used to project laser scans into point clouds

    std::vector<geometry_msgs::msg::Point> transformed_footprint_;
    std::vector<std::shared_ptr<message_filters::SubscriberBase> > observation_subscribers_;  ///< @brief Used for the observation message filters
    std::vector<std::shared_ptr<tf2_ros::MessageFilterBase> > observation_notifiers_;  ///< @brief Used to make sure that transforms are available for each sensor
    std::vector<std::shared_ptr<nav2_costmap_2d::ObservationBuffer> > observation_buffers_;  ///< @brief Used to store observations from various sensors

    double max_obstacle_height_;  ///< @brief Max Obstacle Height
    bool rolling_window_;
    bool was_reset_;
    bool footprint_clearing_enabled_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr mark_sensors_obstacle_service_;
    bool enb_mark_{false};
    rclcpp::Time mark_start_time_;
    bool mark_done_{false};
};

}

#endif // NAV2_COSTMAP_2D__TEMPORARY_OBSTACLE_LAYER2_HPP_
