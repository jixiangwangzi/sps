#include <cartographer_ros/interface_ros.h>
#include "cartographer/mapping/map_builder.h"

#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include "cartographer_ros/occupancy_grid_node.h"

#include "nav_msgs/msg/map_meta_data.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace cartographer_ros {

cartographer_ros::OccupancyGridNode* occ_grid_node_ = NULL;
cartographer_ros::Node* carto_node_ = NULL;
cartographer_ros::NodeOptions node_options_;
cartographer_ros::TrajectoryOptions trajectory_options_;

/*CartographerRosInterface::CartographerRosInterface(std::shared_ptr<tf2_ros::Buffer> buffer)
                    // : tf_buffer_(buffer)
{      
    tf_buffer_ = buffer;
    CHECK(!FLAGS_configuration_directory.empty()) << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty()) << "-configuration_basename is missing.";
    occ_grid_node_ = new cartographer_ros::OccupancyGridNode(FLAGS_resolution, FLAGS_publish_period_sec);

    std::tie(node_options_, trajectory_options_) = LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);
     RCLCPP_INFO(node_handle_->get_logger(), "GingerInterface: cartographer config path: %s/%s", FLAGS_configuration_directory.c_str(), 
            FLAGS_configuration_basename.c_str());

    auto map_builder_ = cartographer::mapping::CreateMapBuilder(node_options_.map_builder_options);
    carto_node_ = new Node(node_options_, std::move(map_builder_), tf_buffer_.get(), FLAGS_collect_metrics);
    if (!FLAGS_load_state_filename.empty()) {
        carto_node_->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }

    if (FLAGS_start_trajectory_with_default_topics) {
        carto_node_->StartTrajectoryWithDefaultTopics(trajectory_options_);
    }
}*/


CartographerRosInterface::CartographerRosInterface(const rclcpp::Node::SharedPtr node_handle, std::shared_ptr<tf2_ros::Buffer> buffer, const int& slam_model)
                    : node_handle_(node_handle), tf_buffer_(buffer)
{        
    CHECK(!FLAGS_configuration_directory.empty())
        << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
        << "-configuration_basename is missing.";
    if (5 == slam_model) {
        std::tie(node_options_, trajectory_options_) =
            LoadOptions(FLAGS_configuration_cloud_directory,FLAGS_configuration_cloud_basename,FLAGS_configuration_directory, "cartographer_odom_calib.lua");
        RCLCPP_INFO(node_handle_->get_logger(), "GingerInterface: cartographer config path: %s/%s", FLAGS_configuration_directory.c_str(), 
                "cartographer_odom_calib.lua");
    }
    else {
        std::tie(node_options_, trajectory_options_) =
            LoadOptions(FLAGS_configuration_cloud_directory,FLAGS_configuration_cloud_basename,FLAGS_configuration_directory, FLAGS_configuration_basename);
        RCLCPP_INFO(node_handle_->get_logger(), "GingerInterface: cartographer config path: %s/%s", FLAGS_configuration_directory.c_str(), 
                FLAGS_configuration_basename.c_str());
    }

    auto map_builder_ =
        cartographer::mapping::CreateMapBuilder(node_options_.map_builder_options);
    carto_node_ = new Node(node_options_, node_handle_, std::move(map_builder_), tf_buffer_.get(),
                FLAGS_collect_metrics, trajectory_options_);
    carto_node_->setFlagIsLocalization(false);
    if (!FLAGS_load_state_filename.empty()) {
        carto_node_->LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
    }
    if(slam_model == 1 || 5 == slam_model){ //MODE_MAPPING
        carto_node_->setInitFinish(true);
        LOG(INFO) << "slam model is MODE_MAPPING";
        if (FLAGS_start_trajectory_with_default_topics) {
            carto_node_->StartTrajectoryWithDefaultTopics(trajectory_options_);
        }
    }
    else{
        LOG(INFO) << "slam model is " << slam_model;
        carto_node_->setInitFinish(false);
    }
    // if (FLAGS_start_trajectory_with_default_topics) {
    //     carto_node_->StartTrajectoryWithDefaultTopics(trajectory_options_);
    // }
    occ_grid_node_ = new cartographer_ros::OccupancyGridNode( FLAGS_resolution, FLAGS_publish_period_sec);
}

CartographerRosInterface::~CartographerRosInterface()
{
    if(occ_grid_node_)
    {
        delete occ_grid_node_;
        occ_grid_node_ = NULL;
    } 
    if (carto_node_)
    {
        // carto_node_->FinishAllTrajectories();
        // carto_node_->RunFinalOptimization();
        delete carto_node_;
        carto_node_ = NULL;
    }


}

bool CartographerRosInterface::SetOccMap(const nav_msgs::msg::OccupancyGrid& occ_map) 
{
    return carto_node_->SetOccMap(occ_map);
}

int CartographerRosInterface::SetInitialPose(const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& initial_pose)
{
    return carto_node_->SetInitialPose(initial_pose);
}
bool CartographerRosInterface::SetFlagMatchMap(const bool& msg){
    return carto_node_->SetFlagMatchMap(msg);
}
bool CartographerRosInterface::SetRelocalizationState(const int& reloc_state)
{
    return carto_node_->SetRelocalizationState(reloc_state);
}

bool CartographerRosInterface::naviTypeCB(const int& naviType){
    return carto_node_->naviTypeCB(naviType);
}

double CartographerRosInterface::GetLocConfidence()
{
    return carto_node_->GetLocConfidence();
}

bool CartographerRosInterface::GetFlagInWall()
{
    return carto_node_->GetFlagInWall();
}

std::unique_ptr<nav_msgs::msg::OccupancyGrid> CartographerRosInterface::GetOccMap()
{
    if (!occ_grid_node_) return nullptr;
    return occ_grid_node_->DrawOccGrid();
}


}
