/*********************************************************************
 * Author: Jate Xiong
 *********************************************************************/
#ifndef NAV2_COSTMAP_2D__VIRTUAL_LAYER_HPP_
#define NAV2_COSTMAP_2D__VIRTUAL_LAYER_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "message_filters/subscriber.h"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"

#include "cm_msgs/msg/virtualwalls.hpp"

namespace nav2_costmap_2d
{

struct Point2DInt
{
 unsigned int x;
 unsigned int y;
};

class VirtualWallLayer : public CostmapLayer
{
public:
    VirtualWallLayer();
    virtual ~VirtualWallLayer();
    virtual void onInitialize();
    virtual void activate();
    virtual void deactivate();
    virtual void reset();
    /**
     * @brief If clearing operations should be processed on this layer or not
     */
    virtual bool isClearable() {return false;}    
    virtual void updateBounds(double robot_x, double robot_y,
                              double robot_yaw, double* min_x, double* min_y,
                              double* max_x, double* max_y);
    virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid,
                             int min_i, int min_j, int max_i, int max_j);
    virtual void matchSize();
private:
    /**
     * @brief Get parameters of layer
     */
    void getParameters();

    /**
     * @brief Process a new map coming from a topic
     */
    void processVirtualwalls(const cm_msgs::msg::Virtualwalls & new_virtualwalls);
    void incomingVirtualwalls(const cm_msgs::msg::Virtualwalls::SharedPtr new_virtualwalls);
    // void reconfigureCB(nav2_costmap_2d::GenericPluginConfig &config, uint32_t level);
    void insertOneVirtualWall(double start_x, double start_y, double end_x, double end_y);
    void insertAllVirtualWalls(const cm_msgs::msg::Virtualwalls &virtualwalls);
    void virtualWalls2map(double start_x, double start_y, double end_x, double end_y);
    void getLineCells(int x0, int y0, int x1, int y1, std::vector<Point2DInt> &cells);
    /**
     * @brief Callback executed when a parameter change is detected
     * @param event ParameterEvent message
     */
    rcl_interfaces::msg::SetParametersResult
    dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

private:
    // ros::Subscriber virtualwall_sub_;
    rclcpp::Subscription<cm_msgs::msg::Virtualwalls>::SharedPtr virtualwall_sub_;
    cm_msgs::msg::Virtualwalls virtualwalls_;

    
    /// @brief frame that map is located in
    std::string virtualwall_topic_;
    bool virtualwall_received_{false};
    bool virtualwall_subscribe_transient_local_;

    bool has_updated_data_{false};
    unsigned int x_{0};
    unsigned int y_{0};
    unsigned int width_{0};
    unsigned int height_{0};
    unsigned int min_x_{0}, min_y_{0}, max_x_{0}, max_y_{0};


    // dynamic_reconfigure::Server<nav2_costmap_2d::GenericPluginConfig> *dsrv_;
    bool rolling_window_;
    std::string map_frame_;
    std::string global_frame_;
    // bool update_costmap_;
    tf2::Duration transform_tolerance_;
    std::atomic<bool> update_in_progress_;
    cm_msgs::msg::Virtualwalls::SharedPtr virtualwalls_buffer_;
    // Dynamic parameters handler
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    rclcpp::Clock::SharedPtr clock_;  
};

}  // namespace nav2_costmap_2d

#endif  // NAV2_COSTMAP_2D__VIRTUAL_LAYER_HPP_

