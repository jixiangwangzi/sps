#include <cartographer_ros/interface_loc_ros.h>
#include "cartographer/mapping/map_builder.h"

#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include "cartographer_ros/occupancy_grid_node.h"

// #include "nav_msgs/msg/MapMetaData.h"
#include "sensor_msgs/msg/laser_scan.hpp"


namespace cartographer_ros {

cartographer_ros::Node* carto_loc_node_ = NULL;
cartographer_ros::NodeOptions loc_node_options_;
cartographer_ros::TrajectoryOptions loc_trajectory_options_;

CartographerLocRosInterface::CartographerLocRosInterface(const rclcpp::Node::SharedPtr node_handle, std::shared_ptr<tf2_ros::Buffer> buffer)
                    : node_handle_(node_handle), tf_buffer_(buffer)
{   
    LOG(INFO) << "CartographerLocRosInterface::CartographerLocRosInterface() start!";
    CHECK(!FLAGS_configuration_directory.empty())
        << "-configuration_directory is missing.";
    CHECK(!FLAGS_configuration_basename.empty())
        << "-configuration_basename is missing.";

    std::tie(loc_node_options_, loc_trajectory_options_) =
        LoadOptions(FLAGS_configuration_cloud_directory,FLAGS_configuration_cloud_basename,FLAGS_configuration_directory, "cartographer_loc.lua");
    LOG(INFO) <<"GingerInterface: cartographer config path: " << FLAGS_configuration_directory.c_str() <<
            "cartographer_loc.lua";
    auto map_builder_ =
        cartographer::mapping::CreateMapBuilder(loc_node_options_.map_builder_options);

    carto_loc_node_ = new Node(loc_node_options_, node_handle_, std::move(map_builder_), tf_buffer_.get(),
                FLAGS_collect_metrics, loc_trajectory_options_);
    carto_loc_node_->setFlagIsLocalization(true);
    LOG(INFO) <<"CartographerLocRosInterface() end !";

    // ros::AsyncSpinner spinner(2);
    // spinner.start();
    // while(!carto_loc_node_->getInitFinish() && ros::ok()){
    //     ros::WallDuration(1.0).sleep();
    // }
    // spinner.stop(); 

    // if (FLAGS_start_trajectory_with_default_topics) {
    //     carto_loc_node_->StartTrajectoryWithDefaultTopics(carto_loc_node_->getTrajectory_options());
    // }
}

CartographerLocRosInterface::~CartographerLocRosInterface()
{
    if (carto_loc_node_)
    {
        // carto_loc_node_->FinishAllTrajectories();
        // carto_loc_node_->RunFinalOptimization();
        delete carto_loc_node_;
        carto_loc_node_ = NULL;
    }


}

bool CartographerLocRosInterface::SetOccMap(const nav_msgs::msg::OccupancyGrid& occ_map) 
{
    return carto_loc_node_->SetOccMap(occ_map);
}

int CartographerLocRosInterface::SetInitialPose(const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& initial_pose)
{
    return carto_loc_node_->SetInitialPose(initial_pose);
}
bool CartographerLocRosInterface::SetFlagMatchMap(const bool& msg){
    return carto_loc_node_->SetFlagMatchMap(msg);
}
bool CartographerLocRosInterface::SetRelocalizationState(const int& reloc_state)
{
    return carto_loc_node_->SetRelocalizationState(reloc_state);
}

bool CartographerLocRosInterface::naviTypeCB(const int& naviType){
    return carto_loc_node_->naviTypeCB(naviType);
}

double CartographerLocRosInterface::GetLocConfidence()
{
    return carto_loc_node_->GetLocConfidence();
}

bool CartographerLocRosInterface::GetFlagInWall()
{
    return carto_loc_node_->GetFlagInWall();
}

double CartographerLocRosInterface::GetVisualConfidence()
{
    return carto_loc_node_->GetVisualConfidence();
}

bool CartographerLocRosInterface::GetJumpPoseFlag()
{
    return carto_loc_node_->GetJumpFlag();
}

bool CartographerLocRosInterface::IsFinishChangeMap()
{
    return carto_loc_node_->CheckSwitchMapFinish();
}

bool CartographerLocRosInterface::IsBadScanData()
{
    return carto_loc_node_->IsBadScanData();
}

bool CartographerLocRosInterface::UseVisualFlag()
{
    return carto_loc_node_->UseVisualFlag();
}
bool CartographerLocRosInterface::UseConfidence()
{
    return carto_loc_node_->UseConfidence();
}

bool CartographerLocRosInterface::GetLaserLostFlag()
{
    return carto_loc_node_->GetLaserLostFlag();
}

bool CartographerLocRosInterface::GetOdomLostFlag()
{
    return carto_loc_node_->GetOdomLostFlag();
}

bool CartographerLocRosInterface::GetInitFinishFlag()
{
    return carto_loc_node_->getInitFinish();
}

double CartographerLocRosInterface::GetLaserTh()
{
    return carto_loc_node_->GetLaserTh();
}

double CartographerLocRosInterface::GetVisualTh()
{
    return carto_loc_node_->GetVisualTh();
}

double CartographerLocRosInterface::GetCancelTh()
{
    return carto_loc_node_->GetCancelTh();
}

}
