#ifndef SENSOR_FILTER_H_
#define SENSOR_FILTER_H_

#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
//#include "nav2_lifecycle_manager/lifecycle_manager_client.hpp"
#include "laser_filter.h"

#include <sensor_msgs/msg/laser_scan.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>




//#include "sensor_vel_filters/include/pointcloud_filter.h"

namespace sensor_vel_filters
{

class SensorFilter :  public nav2_util::LifecycleNode
{

public:
    SensorFilter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~SensorFilter();

protected:
    /**
     * @brief Configures member variables
     *
     * Initializes action server for "NavigationToPose"; subscription to
     * "goal_sub"; and builds behavior tree from xml file.
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Activates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Deactivates action server
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Resets member variables
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
    /**
     * @brief Called when in shutdown state
     * @param state Reference to LifeCycle node state
     * @return SUCCESS or FAILURE
     */
    nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    //void SensordataThreadLoop();

private:
    //std::vector<std::shared_ptr<LaserScanFilter>> laserscan_filters_;
    std::vector<std::shared_ptr<LaserScanFilter>> laserscan_filters_;
    //std::vector<std::shared_ptr<PointCloudFilter>> pointcloud_filters_;
    bool process_active_;
    rclcpp::Clock::SharedPtr clock_;
    //std::thread sensordata_thread_;
    std::string node_name_;
};

} // namespace sensor_vel_filters

#endif