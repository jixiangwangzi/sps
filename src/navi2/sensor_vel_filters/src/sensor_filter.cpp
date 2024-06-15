#include "sensor_filter.h"

using namespace std;
//using namespace sensor_vel_filters;
using namespace std::chrono_literals;

namespace sensor_vel_filters{


SensorFilter::SensorFilter(const rclcpp::NodeOptions &options)
      : nav2_util::LifecycleNode("sensor_filter", "", true, options),
        process_active_(false)
{
    clock_ = this->get_clock();
    RCLCPP_INFO(get_logger(), "sensor_filter Creating");
}

SensorFilter::~SensorFilter()
{
    //sensordata_thread_.join();
    laserscan_filters_.clear();
    //pointcloud_filters_.clear();
}

nav2_util::CallbackReturn SensorFilter::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "sensor_filter Configuring");
    try{
        auto node = shared_from_this();
        nav2_util::declare_parameter_if_not_declared(
            node, "filters", rclcpp::ParameterValue(std::vector<std::string>())); 
        std::vector<std::string> filter_names = get_parameter("filters").as_string_array();  
        for(std::string filter_name : filter_names){
            nav2_util::declare_parameter_if_not_declared(
                node, filter_name + ".type", rclcpp::PARAMETER_STRING);   
            const std::string filter_type = get_parameter(filter_name + ".type").as_string();

            if(filter_type == "LaserScan"){
                std::shared_ptr<LaserScanFilter> s = std::make_shared<LaserScanFilter>(node,filter_name);
                s->configure();
                laserscan_filters_.push_back(s);
            // }else if(filter_type == "PointCloud"){
            //     std::shared_ptr<PointCloudFilter> s = std::make_shared<PointCloudFilter>(node,filter_name);
            //     s->configure();
            }else{
                RCLCPP_ERROR(
                get_logger(),
                "configure FAILURE;");    
                return nav2_util::CallbackReturn::FAILURE;         
            }
        }    
    }catch(const std::exception & ex){
        RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
        return nav2_util::CallbackReturn::FAILURE;
    }

    //sensordata_thread_ = std::thread(std::bind(&SensorFilter::SensordataThreadLoop, this));

    return nav2_util::CallbackReturn::SUCCESS;
}
/**
 * @brief Activates action server
 * @param state Reference to LifeCycle node state
 * @return SUCCESS or FAILURE
 */
nav2_util::CallbackReturn SensorFilter::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "sensor_filter Activating");

    for(std::shared_ptr<LaserScanFilter> laserscan_filter : laserscan_filters_){
        laserscan_filter->activate();
    }
    // Activating main worker
    process_active_ = true;
    // Creating bond connection
    createBond();
    return nav2_util::CallbackReturn::SUCCESS;
}
/**
 * @brief Deactivates action server
 * @param state Reference to LifeCycle node state
 * @return SUCCESS or FAILURE
 */
nav2_util::CallbackReturn SensorFilter::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "sensor_filter Deactivating");
    for(std::shared_ptr<LaserScanFilter> laserscan_filter : laserscan_filters_){
        laserscan_filter->deactivate();
    }
    // Deactivating main worker
    process_active_ = false;
    // Destroying bond connection
    destroyBond();
    return nav2_util::CallbackReturn::SUCCESS;
}
/**
 * @brief Resets member variables
 * @param state Reference to LifeCycle node state
 * @return SUCCESS or FAILURE
 */
nav2_util::CallbackReturn SensorFilter::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "sensor_filter Cleaning up");
    return nav2_util::CallbackReturn::SUCCESS;
}
/**
 * @brief Called when in shutdown state
 * @param state Reference to LifeCycle node state
 * @return SUCCESS or FAILURE
 */
nav2_util::CallbackReturn SensorFilter::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(get_logger(), "sensor_filter Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
}

// void SensorFilter::SensordataThreadLoop()
// {
//     rclcpp::Rate rate(15.0);
//     while(rclcpp::ok()){
//         for (std::shared_ptr<LaserScanFilter> laserscan_filter : laserscan_filters_) {
//             laserscan_filter->publish();
//         }
//         // for (std::shared_ptr<PointCloudFilter> pointcloud_filter : pointcloud_filters_) {
//         //     pointcloud_filter->publish();
//         // }
//         rate.sleep();
//     }
// }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(sensor_vel_filters::SensorFilter)
