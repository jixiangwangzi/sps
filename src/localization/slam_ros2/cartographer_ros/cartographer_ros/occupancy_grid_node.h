#ifndef CARTOGRAPHER_ROS_OCCUPANCY_GRID_NODE_H
#define CARTOGRAPHER_ROS_OCCUPANCY_GRID_NODE_H

#include <cmath>
#include <string>
#include <vector>
#include <mutex>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "absl/synchronization/mutex.h"
#include "cairo/cairo.h"
#include "cartographer/common/port.h"
#include "cartographer/io/image.h"
#include "cartographer/io/submap_painter.h"
#include "cartographer/mapping/id.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/ros_log_sink.h"
#include "cartographer_ros/submap.h"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "gflags/gflags.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <rclcpp/rclcpp.hpp>

// DEFINE_double(resolution, 0.05,
//               "Resolution of a grid cell in the published occupancy grid.");
// DEFINE_double(publish_period_sec, 1.0, "OccupancyGrid publishing period.");
// DEFINE_bool(include_frozen_submaps, true,
//             "Include frozen submaps in the occupancy grid.");
// DEFINE_bool(include_unfrozen_submaps, true,
//             "Include unfrozen submaps in the occupancy grid.");
// DEFINE_string(occupancy_grid_topic, cartographer_ros::kOccupancyGridTopic,
//               "Name of the topic on which the occupancy grid is published.");
#define FLAGS_include_unfrozen_submaps true
#define FLAGS_include_frozen_submaps true
#define FLAGS_occupancy_grid_topic cartographer_ros::kOccupancyGridTopic

namespace cartographer_ros {

using ::cartographer::io::PaintSubmapSlicesResult;
using ::cartographer::io::SubmapSlice;
using ::cartographer::mapping::SubmapId;

class OccupancyGridNode : public rclcpp::Node {
 public:
  explicit OccupancyGridNode(const double resolution, const double publish_period_sec);
//   explicit OccupancyGridNode(rclcpp::Node::SharedPtr node_handle, double resolution, double publish_period_sec);

  ~OccupancyGridNode() {
      RCLCPP_INFO(this->get_logger(),"OccupancyGridNode: ~OccupancyGridNode");
  };

  OccupancyGridNode(const OccupancyGridNode&) = delete;
  OccupancyGridNode& operator=(const OccupancyGridNode&) = delete;

  // std::unique_ptr<nav_msgs::msg::OccupancyGrid> DrawOccupancyGrid();
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> DrawOccGrid();
  std::unique_ptr<nav_msgs::msg::OccupancyGrid> occGrid_ptr_ GUARDED_BY(mutex_);
 private:
  void HandleSubmapList(const cartographer_ros_msgs::msg::SubmapList::SharedPtr msg);
  void DrawAndPublish(); // const ::ros::WallTimerEvent& timer_event

  const double resolution_,publish_period_sec_;
  absl::Mutex mutex_;
  // ::rclcpp::Node::SharedPtr node_handle_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr callback_group_executor_;
  ::rclcpp::Client<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr client_ GUARDED_BY(mutex_);
  ::rclcpp::Subscription<cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_subscriber_ GUARDED_BY(mutex_);
  ::rclcpp::Publisher<::nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_publisher_ GUARDED_BY(mutex_);
  std::map<SubmapId, SubmapSlice> submap_slices_ GUARDED_BY(mutex_);
  rclcpp::TimerBase::SharedPtr occupancy_grid_publisher_timer_;
  std::string last_frame_id_;
  rclcpp::Time last_timestamp_;
};

}  // namespace cartographer_ros

#endif
