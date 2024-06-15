
#include <cartographer_ros/occupancy_grid_node.h>


namespace cartographer_ros {

OccupancyGridNode::OccupancyGridNode(const double resolution, const double publish_period_sec)
    : rclcpp::Node("cartographer_occupancy_grid_node"),
      resolution_(resolution), publish_period_sec_(publish_period_sec)
    {
      callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
      callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      callback_group_executor_->add_callback_group(callback_group_, this->get_node_base_interface());
      client_ = this->create_client<cartographer_ros_msgs::srv::SubmapQuery>(
            kSubmapQueryServiceName,
            rmw_qos_profile_services_default,
            callback_group_
            );
      submap_list_subscriber_ = create_subscription<cartographer_ros_msgs::msg::SubmapList>(
          kSubmapListTopic, rclcpp::QoS(kLatestOnlyPublisherQueueSize),
          std::bind(&OccupancyGridNode::HandleSubmapList,this,std::placeholders::_1));
    //   ,occupancy_grid_publisher_(
    //       node_handle_.advertise<::nav_msgs::OccupancyGrid>(
    //           FLAGS_occupancy_grid_topic, kLatestOnlyPublisherQueueSize,
    //           true /* latched */))
      // ,occupancy_grid_publisher_timer_(
      //     node_handle_.createWallTimer(::ros::WallDuration(publish_period_sec),
      //                                  &OccupancyGridNode::DrawAndPublish, this)) 
    }
/*
OccupancyGridNode::OccupancyGridNode(rclcpp::Node::SharedPtr node_handle, double resolution, double publish_period_sec)
    : rclcpp::Node("cartographer_occupancy_grid_node"),
      resolution_(resolution),publish_period_sec_(publish_period_sec)
    {
      callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive,
        false);
      callback_group_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      callback_group_executor_->add_callback_group(callback_group_, this->get_node_base_interface());
      client_ = this->create_client<cartographer_ros_msgs::srv::SubmapQuery>(
            kSubmapQueryServiceName,
            rmw_qos_profile_services_default,
            callback_group_
            );
      submap_list_subscriber_ = create_subscription<cartographer_ros_msgs::msg::SubmapList>(
          kSubmapListTopic, rclcpp::QoS(kLatestOnlyPublisherQueueSize),
          std::bind(&OccupancyGridNode::HandleSubmapList,this,std::placeholders::_1));}
*/

void OccupancyGridNode::HandleSubmapList(
    const cartographer_ros_msgs::msg::SubmapList::SharedPtr msg) {
  absl::MutexLock locker(&mutex_);

  // We do not do any work if nobody listens.
  // if (occupancy_grid_publisher_.getNumSubscribers() == 0) {
  //   return;
  // }
  // Keep track of submap IDs that don't appear in the message anymore.
  std::set<SubmapId> submap_ids_to_delete;
  for (const auto& pair : submap_slices_) {
    submap_ids_to_delete.insert(pair.first);
  }
  for (const auto& submap_msg : msg->submap) {
    const SubmapId id{submap_msg.trajectory_id, submap_msg.submap_index};
    submap_ids_to_delete.erase(id);
    if ((submap_msg.is_frozen && !FLAGS_include_frozen_submaps) ||
        (!submap_msg.is_frozen && !FLAGS_include_unfrozen_submaps)) {
      continue;
    }
    SubmapSlice& submap_slice = submap_slices_[id];
    submap_slice.pose = ToRigid3d(submap_msg.pose);
    submap_slice.metadata_version = submap_msg.submap_version;
    if (submap_slice.surface != nullptr &&
        submap_slice.version == submap_msg.submap_version) {
      continue;
    }

    auto fetched_textures = cartographer_ros::FetchSubmapTextures(
          id, client_, callback_group_executor_,
          std::chrono::milliseconds(int(publish_period_sec_ * 1000)));
    if (fetched_textures == nullptr) {
      continue;
    }
    CHECK(!fetched_textures->textures.empty());
    submap_slice.version = fetched_textures->version;

    // We use the first texture only. By convention this is the highest
    // resolution texture and that is the one we want to use to construct the
    // map for ROS.
    const auto fetched_texture = fetched_textures->textures.begin();
    submap_slice.width = fetched_texture->width;
    submap_slice.height = fetched_texture->height;
    submap_slice.slice_pose = fetched_texture->slice_pose;
    submap_slice.resolution = fetched_texture->resolution;
    submap_slice.cairo_data.clear();
    submap_slice.surface = ::cartographer::io::DrawTexture(
        fetched_texture->pixels.intensity, fetched_texture->pixels.alpha,
        fetched_texture->width, fetched_texture->height,
        &submap_slice.cairo_data);
  }

  // Delete all submaps that didn't appear in the message.
  for (const auto& id : submap_ids_to_delete) {
    submap_slices_.erase(id);
  }

  last_timestamp_ = msg->header.stamp;
  last_frame_id_ = msg->header.frame_id;
}

void OccupancyGridNode::DrawAndPublish() {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return;
  }
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  // std::unique_ptr<nav_msgs::msg::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
  //     painted_slices, resolution_, last_frame_id_, last_timestamp_);
  occGrid_ptr_ = CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  // occupancy_grid_publisher_.publish(*msg_ptr);
  return;
}

std::unique_ptr<nav_msgs::msg::OccupancyGrid> OccupancyGridNode::DrawOccGrid() {
  absl::MutexLock locker(&mutex_);
  if (submap_slices_.empty() || last_frame_id_.empty()) {
    return nullptr;
  }
  auto painted_slices = PaintSubmapSlices(submap_slices_, resolution_);
  // std::unique_ptr<nav_msgs::OccupancyGrid> msg_ptr = CreateOccupancyGridMsg(
  //     painted_slices, resolution_, last_frame_id_, last_timestamp_);
  return  CreateOccupancyGridMsg(
      painted_slices, resolution_, last_frame_id_, last_timestamp_);
  // occupancy_grid_publisher_.publish(*msg_ptr);
  // return;
}

}  // namespace cartographer_ros

