/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer_ros/node.h"

#include <chrono>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
//#include "ros/serialization.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/int32.hpp"

DEFINE_bool(collect_metrics, false,
            "Activates the collection of runtime metrics. If activated, the "
            "metrics can be accessed via a ROS service.");
DEFINE_string(configuration_cloud_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_cloud_basename, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_directory, "",
              "First directory in which configuration files are searched, "
              "second is always the Cartographer installation to allow "
              "including files from there.");
DEFINE_string(configuration_basename, "",
              "Basename, i.e. not containing any directory prefix, of the "
              "configuration file.");
DEFINE_string(load_state_filename, "",
              "If non-empty, filename of a .pbstream file to load, containing "
              "a saved SLAM state.");
DEFINE_bool(load_frozen_state, true,
            "Load the saved state as frozen (non-optimized) trajectories.");
DEFINE_bool(
    start_trajectory_with_default_topics, true,
    "Enable to immediately start the first trajectory with default topics.");
DEFINE_string(
    save_state_filename, "",
    "If non-empty, serialize state and write it to disk before shutting down.");
    
DEFINE_double(resolution, 0.05,
              "Resolution of a grid cell in the published occupancy grid.");
DEFINE_double(publish_period_sec, 2.0, "OccupancyGrid publishing period.");
namespace cartographer_ros {

namespace carto = ::cartographer;

using carto::transform::Rigid2d;
using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType>
::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr&),
    const int trajectory_id, const std::string& topic,
    ::rclcpp::Node::SharedPtr node_handle, Node* const node) {
  return node_handle->create_subscription<MessageType>(
      topic, rclcpp::SensorDataQoS(),
      [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
            (node->*handler)(trajectory_id, topic, msg);
          });
}

std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
  }
  return "";
}

}  // namespace


/*Node::Node(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer, const bool collect_metrics)
    : node_options_(node_options),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer, globalMessagePtr_), 
      initFinish_(false) {
  absl::MutexLock lock(&mutex_);
  if (collect_metrics) {
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        node_handle_.advertise<::geometry_msgs::PoseStamped>(
            kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
  }
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kTrajectoryQueryServiceName, &Node::HandleTrajectoryQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kReadMetricsServiceName, &Node::HandleReadMetrics, this));

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_.createTimer(
        ::ros::Duration(node_options_.pose_publish_period_sec),
        &Node::PublishLocalTrajectoryData, this);
  }
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
}

Node::Node(
    const NodeOptions& node_options, const ros::NodeHandle& node_handle,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer, const bool collect_metrics)
    : node_options_(node_options), node_handle_(node_handle),
      globalMessagePtr_(new ::cartographer::GlobalMessage),
      map_builder_bridge_(node_options_, std::move(map_builder), tf_buffer, globalMessagePtr_) {
  absl::MutexLock lock(&mutex_);
  if (collect_metrics) {
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }

  submap_list_publisher_ =
      node_handle_.advertise<::cartographer_ros_msgs::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_.advertise<::visualization_msgs::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        node_handle_.advertise<::geometry_msgs::PoseStamped>(
            kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
  }
  service_servers_.push_back(node_handle_.advertiseService(
      kSubmapQueryServiceName, &Node::HandleSubmapQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kTrajectoryQueryServiceName, &Node::HandleTrajectoryQuery, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kStartTrajectoryServiceName, &Node::HandleStartTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kFinishTrajectoryServiceName, &Node::HandleFinishTrajectory, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kWriteStateServiceName, &Node::HandleWriteState, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kGetTrajectoryStatesServiceName, &Node::HandleGetTrajectoryStates, this));
  service_servers_.push_back(node_handle_.advertiseService(
      kReadMetricsServiceName, &Node::HandleReadMetrics, this));

  scan_matched_point_cloud_publisher_ =
      node_handle_.advertise<sensor_msgs::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);

  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.submap_publish_period_sec),
      &Node::PublishSubmapList, this));
  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_.createTimer(
        ::ros::Duration(node_options_.pose_publish_period_sec),
        &Node::PublishLocalTrajectoryData, this);
  }
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishTrajectoryNodeList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(node_options_.trajectory_publish_period_sec),
      &Node::PublishLandmarkPosesList, this));
  wall_timers_.push_back(node_handle_.createWallTimer(
      ::ros::WallDuration(kConstraintPublishPeriodSec),
      &Node::PublishConstraintList, this));
}
*/
Node::Node(
    const NodeOptions& node_options, rclcpp::Node::SharedPtr node_handle,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* const tf_buffer, const bool collect_metrics, TrajectoryOptions& trajectory_options)
    : node_options_(node_options),
      node_handle_(node_handle),
      tf_broadcaster_(tf2_ros::TransformBroadcaster(node_handle_)),
      globalMessagePtr_(new ::cartographer::GlobalMessage),
    //   map_builder_bridge_.reset(new cartographer_ros::MapBuilderBridge(node_options_, std::move(map_builder), tf_buffer, globalMessagePtr_,node_handle)),
      initFinish_(false), setMapWorking_(false), trajectory_options_(trajectory_options) {
  absl::MutexLock lock(&mutex_);
  globalMessagePtr_->setFlagUseSubmapMatch(trajectory_options.use_submap_match);
  globalMessagePtr_->setSubmapFreeThreshold(trajectory_options.submap_free_threshold);
  if (collect_metrics) {
    metrics_registry_ = absl::make_unique<metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }
  occ_map_ = std::make_shared<nav_msgs::msg::OccupancyGrid>();
  map_builder_bridge_.reset(new cartographer_ros::MapBuilderBridge(node_options_, std::move(map_builder), tf_buffer, globalMessagePtr_,node_handle));
  submap_list_publisher_ =
      node_handle_->create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
          kSubmapListTopic, kLatestOnlyPublisherQueueSize);
  trajectory_node_list_publisher_ =
      node_handle_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kTrajectoryNodeListTopic, kLatestOnlyPublisherQueueSize);
  landmark_poses_list_publisher_ =
      node_handle_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kLandmarkPosesListTopic, kLatestOnlyPublisherQueueSize);
  constraint_list_publisher_ =
      node_handle_->create_publisher<::visualization_msgs::msg::MarkerArray>(
          kConstraintListTopic, kLatestOnlyPublisherQueueSize);
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        node_handle_->create_publisher<::geometry_msgs::msg::PoseStamped>(
            kTrackedPoseTopic, kLatestOnlyPublisherQueueSize);
  }
  occ_map_publisher_ =  node_handle_->create_publisher<::nav_msgs::msg::OccupancyGrid>(
       "occMap", kLatestOnlyPublisherQueueSize);
  scan_matched_point_cloud_publisher_ = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>(
          kScanMatchedPointCloudTopic, kLatestOnlyPublisherQueueSize);
  submap_point_cloud_publisher_ = node_handle_->create_publisher<sensor_msgs::msg::PointCloud2>("/submap_point_cloud", 2);
  // initial_pose_sub_ = node_handle_.subscribe("initialpose", 1, &Node::initialPoseCallback, this);

  pub_loc_init_finish_ = node_handle_->create_publisher<std_msgs::msg::Int32>("/loc_init_flag",5);
  initial_state_publisher_ = node_handle->create_publisher<sps_common_msgs::msg::InitialSlamState>("/initial_slam_state",5);

  submap_query_server_ = node_handle_->create_service<cartographer_ros_msgs::srv::SubmapQuery>(
      kSubmapQueryServiceName, std::bind(&Node::HandleSubmapQuery, this, std::placeholders::_1, std::placeholders::_2));
  trajectory_query_server = node_handle_->create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
      kTrajectoryQueryServiceName, std::bind(&Node::HandleTrajectoryQuery, this, std::placeholders::_1, std::placeholders::_2));
  start_trajectory_server_ = node_handle_->create_service<cartographer_ros_msgs::srv::StartTrajectory>(
      kStartTrajectoryServiceName, std::bind(&Node::HandleStartTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  finish_trajectory_server_ = node_handle_->create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
      kFinishTrajectoryServiceName, std::bind(&Node::HandleFinishTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  write_state_server_ = node_handle_->create_service<cartographer_ros_msgs::srv::WriteState>(
      kWriteStateServiceName, std::bind(&Node::HandleWriteState, this, std::placeholders::_1, std::placeholders::_2));
  get_trajectory_states_server_= node_handle_->create_service<cartographer_ros_msgs::srv::GetTrajectoryStates>(
      kGetTrajectoryStatesServiceName, std::bind(&Node::HandleGetTrajectoryStates, this, std::placeholders::_1, std::placeholders::_2));
  read_metrics_server_ = node_handle_->create_service<cartographer_ros_msgs::srv::ReadMetrics>(
      kReadMetricsServiceName, std::bind(&Node::HandleReadMetrics, this, std::placeholders::_1, std::placeholders::_2));

  // wall_timers_.push_back(node_handle_->create_wall_timer( std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
  //     [this]() { PublishSubmapList(); }));
  if (node_options_.pose_publish_period_sec > 0) {
    publish_local_trajectory_data_timer_ = node_handle_->create_wall_timer( std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
      [this]() { PublishLocalTrajectoryData(); });
  }
  wall_timers_.push_back(node_handle_->create_wall_timer( std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
      [this]() { PublishTrajectoryNodeList(); }));

  wall_timers_.push_back(node_handle_->create_wall_timer( std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
      [this]() { PublishLandmarkPosesList(); }));

  wall_timers_.push_back(node_handle_->create_wall_timer( std::chrono::milliseconds(int(kConstraintPublishPeriodSec * 1000)),
      [this]() { PublishConstraintList(); }));

  wall_timers_.push_back(node_handle_->create_wall_timer( std::chrono::milliseconds(int(2 * 1000)),
      [this]() { PublishOccMap(); }));

  wall_timers_.push_back(node_handle_->create_wall_timer( std::chrono::milliseconds(int(0.05 * 1000)),
      [this]() { PubLocInitFinish(); }));

  setMapWorking_ = false;
}

Node::~Node() { 
  publish_local_trajectory_data_timer_->cancel();
  for(auto timer_ : wall_timers_){
    timer_->cancel();
  }
  subscribers_.clear();
  rclcpp::Rate(10.).sleep(); // hz 
  // using namespace std::chrono_literals;
  // std::this_thread::sleep_for(100ms);
  int numTrajectory = map_builder_bridge_->GetTrajectoryStates().size();
  for (int i = numTrajectory - 1; i >= 0; i--) {
      RCLCPP_INFO(node_handle_->get_logger(), "Cartographer node: finish all trajectory %d", i);
      map_builder_bridge_->DeleteTrajectory(i);
  }
  extrapolators_.clear();
  sensor_samplers_.clear();  
  // FinishAllTrajectories(); 
  RCLCPP_INFO(node_handle_->get_logger(), "NOde:: ~Node()");
}

rclcpp::Node::SharedPtr Node::node_handle() { return node_handle_; }

bool Node::HandleSubmapQuery(
    const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->HandleSubmapQuery(request, response);
  return true;
}

bool Node::HandleTrajectoryQuery(
    const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = TrajectoryStateToStatus(
      request->trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response->status.message;
    return true;
  }
  map_builder_bridge_->HandleTrajectoryQuery(request, response);
  return true;
}

void Node::PublishSubmapList() {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_->GetSubmapList());
}

void Node::AddExtrapolator(const int trajectory_id,
                           const TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Node::PublishLocalTrajectoryData() {
  absl::MutexLock lock(&mutex_);
  if(!initFinish_){
    RCLCPP_WARN_THROTTLE(node_handle_->get_logger(),*(node_handle_->get_clock()),5000, "init pose not finish");
  }
  else
  {
    if(node_handle_->now().seconds() - last_stamp_laser_ > 1.0)
    {
      laser_lost_ = true;
      RCLCPP_WARN_THROTTLE(node_handle_->get_logger(),*(node_handle_->get_clock()),1000, "LaserScan data lose");
    }
    else
    {
      laser_lost_ = false;
    }


    if(node_handle_->now().seconds() - last_stamp_odom_ > 1.0)
    {
      odom_lost_ = true;
      RCLCPP_WARN_THROTTLE(node_handle_->get_logger(),*(node_handle_->get_clock()),1000, "odom data lose");
    }
    else
    {
      odom_lost_ = false;
    }
  }
  for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_data.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      if(submap_point_cloud_publisher_->get_subscription_count() > 0)
      {
        carto::sensor::TimedPointCloud submap_timed_point_cloud;
        auto submap_point_cloud = globalMessagePtr_->getTimedSubmapPointcloud();
        if( submap_point_cloud.second != nullptr)
        {
            submap_timed_point_cloud.reserve(submap_point_cloud.second->size());
            for(auto point: submap_point_cloud.second->points())
            {
                submap_timed_point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
                    point, 0.f));

            }
            submap_point_cloud_publisher_->publish(ToPointCloud2Message(carto::common::ToUniversal(submap_point_cloud.first),
                trajectory_options_.tracking_frame, submap_timed_point_cloud));
        }
      }
      if (scan_matched_point_cloud_publisher_->get_subscription_count() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                                .returns.size());
        for (const cartographer::sensor::RangefinderPoint & point :
             trajectory_data.local_slam_data->range_data_in_local.returns) {
          point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
              point, 0.f /* time */));
        }
        scan_matched_point_cloud_publisher_->publish(ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),
            node_options_.map_frame,
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_data.local_to_map.cast<float>())));
      }
      extrapolator.AddPose(trajectory_data.local_slam_data->time,
                           trajectory_data.local_slam_data->local_pose);
    }

    geometry_msgs::msg::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = FromRos(node_handle_->now() + rclcpp::Duration::from_seconds(0.00));
    // std::max(
    //     FromRos(ros::Time::now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp = node_handle_->now();
        // node_options_.use_pose_extrapolator
        //     ? ToRos(now) : ToRos(trajectory_data.local_slam_data->time);
    
    // Suppress publishing if we already published a transform at this time.
    // Due to 2020-07 changes to geometry2, tf buffer will issue warnings for
    // repeated transforms with the same timestamp.
    if (last_published_tf_stamps_.count(entry.first) &&
        last_published_tf_stamps_[entry.first] == stamped_transform.header.stamp)
      continue;
    last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp;

    const Rigid3d tracking_to_local_3d =
        node_options_.use_pose_extrapolator
            ? extrapolator.ExtrapolatePose(now)
            : trajectory_data.local_slam_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));
      }
      return tracking_to_local_3d;
    }();

    const Rigid3d tracking_to_map = trajectory_data.local_to_map * tracking_to_local;
    if(occ_map_ != nullptr){
      Eigen::Matrix<int, 2, 1> translation_tracking_to_origin((int)((tracking_to_map.translation()[0] + translation_map_to_origin_[0]) / 0.05),
                                                    (int)((tracking_to_map.translation()[1] + translation_map_to_origin_[1]) / 0.05));
      unsigned int index_map_ = translation_tracking_to_origin[1] * occ_map_->info.width  + translation_tracking_to_origin[0];
      int occValue_cur = -1;
      if(index_map_ >= occ_map_->data.size()){
        globalMessagePtr_->setFlagInWall(true);
      }
      else{
        occValue_cur = occ_map_->data[translation_tracking_to_origin[1] * occ_map_->info.width  + translation_tracking_to_origin[0]];
      }
      if(occValue_cur != 0){
        globalMessagePtr_->setFlagInWall(true);
      }
      else{
        globalMessagePtr_->setFlagInWall(false);
      }
    }

  // translation_map_to_origin_;
  // occ_map.data[i * occ_map.info.width + j] != 0;


    if (trajectory_data.published_to_tracking != nullptr) {
      if (node_options_.publish_to_tf) {
        if (trajectory_data.trajectory_options.provide_odom_frame) {
          std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;

          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.transform =
              ToGeometryMsgTransform(trajectory_data.local_to_map);
          stamped_transforms.push_back(stamped_transform);

          stamped_transform.header.frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(stamped_transform);
          tf_broadcaster_.sendTransform(stamped_transforms);
        } else {
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          tf_broadcaster_.sendTransform(stamped_transform);
          // LOG(INFO) << "map -> base_link : \n" << tracking_to_map.translation()
          //   << " \n rot: \n" << tracking_to_map.DebugString();
          double diff_trans = (tracking_to_map.translation() - last_tracking_to_map_.translation()).norm();
          double diff_angle = carto::transform::GetYaw(tracking_to_map.rotation()) - carto::transform::GetYaw(last_tracking_to_map_.rotation());
          
          // 位姿跳变
          if(diff_trans >= 0.8)
          {
            pose_jump_flag_ = true;
          }
          else
          {
            pose_jump_flag_ = false;
          }

          if(diff_trans > 0.3 || ((diff_angle * 180 / 3.14) > 20. && (diff_angle * 180 / 3.14) < 350.)){
            RCLCPP_WARN(node_handle_->get_logger(),"diff_trans: %f m, diff_angle: %f degree, time_diff: %f s \n",diff_trans, diff_angle * 180 / 3.14,
              static_cast<double>(stamped_transform.header.stamp.sec) + static_cast<double>(stamped_transform.header.stamp.nanosec) * 1e-9 - 
              static_cast<double>(last_stamped_transform_.header.stamp.sec) - static_cast<double>(last_stamped_transform_.header.stamp.nanosec) * 1e-9);
            LOG(INFO) << "last_tracking_to_map: \n" << last_tracking_to_map_.translation() << std::endl;
            LOG(INFO) << "tracking_to_map: \n" << tracking_to_map.translation() << std::endl;
            LOG(INFO) << "diff_trans: \n" << tracking_to_map.translation() - last_tracking_to_map_.translation() << std::endl;
          }
          last_stamped_transform_ = stamped_transform;
          last_tracking_to_map_ = tracking_to_map;
          // RCLCPP_INFO(node_handle_->get_logger(), "poseSubmap score: %f  poseMap: score: %f", *(map_builder_bridge_->getScorePoseFromSubmapPtr()),
          //   *(map_builder_bridge_->getScorePoseFromMapPtr()));
        }
      }
      if (node_options_.publish_tracked_pose) {
        ::geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = node_options_.map_frame;
        pose_msg.header.stamp = stamped_transform.header.stamp;
        pose_msg.pose = ToGeometryMsgPose(tracking_to_map);
        tracked_pose_publisher_->publish(pose_msg);
      }
    }
    else{
      RCLCPP_WARN(node_handle_->get_logger(), "trajectory_data.published_to_tracking = nullptr");
      last_stamped_transform_.header.stamp = stamped_transform.header.stamp;
      tf_broadcaster_.sendTransform(last_stamped_transform_);
    }
  }
}

void Node::PublishTrajectoryNodeList() {
  if (trajectory_node_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_->publish(
        map_builder_bridge_->GetTrajectoryNodeList());
  }
}

void Node::PublishLandmarkPosesList() {
  if (landmark_poses_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_->publish(
        map_builder_bridge_->GetLandmarkPosesList());
  }
}

void Node::PublishConstraintList() {
  if (constraint_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_->publish(map_builder_bridge_->GetConstraintList());
  }
}

void Node::PublishOccMap() {
  if (occ_map_publisher_->get_subscription_count() > 0 && occ_map_ != nullptr) {
    occ_map_->header.stamp = node_handle_->now();
    occ_map_publisher_->publish(
        *occ_map_);
  }
}

void Node::PubLocInitFinish()
{  
    // if (globalMessagePtr_->getInitState() == cartographer::InitState::BUILD_CONSTRAINT &&
    //     node_handle_->now().seconds() - last_stamp_init_pose_ > 10.0 ) {
    //       globalMessagePtr_->setInitState(cartographer::InitState::FAILED);
    //       LOG(ERROR) << "Build constraint waste time than 10 seconds, Set Init State: FAILED.";
    //     }

    std_msgs::msg::Int32 flagData;
    static int tmp_data =0;

    if (globalMessagePtr_->getFlagInitConstraint()) {
      flagData.data = 1;
      if (globalMessagePtr_->getInitState() == cartographer::InitState::BUILD_CONSTRAINT) {
        LOG(INFO) <<"Build constraint finished, set Init State: SUCCESS.";
        globalMessagePtr_->setInitState(cartographer::InitState::SUCCESS); // 
      }

    } else {
      flagData.data = 0;
    }
    
    if(tmp_data!=flagData.data)
    {
        tmp_data = flagData.data;
        RCLCPP_INFO_STREAM(node_handle_->get_logger(), "\033[36m >>>>>loc init flag change <<<<<\033[0m");
    }

    pub_loc_init_finish_->publish(flagData);
    // publish initial state
    sps_common_msgs::msg::InitialSlamState state_msg;
    state_msg.initial_state = globalMessagePtr_->getInitState();
    initial_state_publisher_->publish(state_msg);

}


// int Node::loadMap(const std::string &path) {
//   map_builder_bridge_->LoadMap(path, occ_map_);
//   return 0;
// }

bool Node::reset() {
  publish_local_trajectory_data_timer_->cancel();
  rclcpp::Rate(5.).sleep(); // hz
  subscribers_.clear();

  int trajectory_num = map_builder_bridge_->GetTrajectoryNum();
  RCLCPP_INFO(node_handle_->get_logger(), "trajectory number: %d",trajectory_num);
  for (int i = 0; i < trajectory_num; i++)
  {
    map_builder_bridge_->DeleteTrajectory(i);
  }
  
  extrapolators_.clear();
  sensor_samplers_.clear();
  RCLCPP_INFO(node_handle_->get_logger(), "cartographer: clear");

  return true;
}

bool Node::SetOccMap(const nav_msgs::msg::OccupancyGrid& occ_map){
  //FinishAllTrajectories();DeleteTrajectory
  absl::MutexLock lock(&setOccMapMutex_);
  globalMessagePtr_->setFlagInsertSubMapID1(false);
  globalMessagePtr_->setInitState(cartographer::InitState::NOT_INITIALZED);
  LOG(INFO) << "cartographer: start set occMap";
  if(setMapWorking_){
    LOG(WARNING) << "cartographer: last set occMap is working!";
    return false;
  }
  setMapWorking_ = true;
  reset();

 // occ_map_ = std::make_shared<nav_msgs::OccupancyGrid>();
  occ_map_->info.resolution = occ_map.info.resolution;
  occ_map_->info.origin.position.x = occ_map.info.origin.position.x;
  occ_map_->info.origin.position.y = occ_map.info.origin.position.y;
  occ_map_->info.origin.orientation = occ_map.info.origin.orientation;
  occ_map_->info.width = occ_map.info.width;
  occ_map_->info.height = occ_map.info.height;
  occ_map_->data = occ_map.data;
  translation_map_to_origin_ << -occ_map.info.origin.position.x, -occ_map.info.origin.position.y;
  occ_map_->header.frame_id = "map";
  occ_map_->header.stamp = node_handle_->now();
  bool result = map_builder_bridge_->SetOccMap(*occ_map_);
  // subscribe vslam pose
  int map_index = map_builder_bridge_->GetTrajectoryNum() - 1;
  CHECK_EQ(map_index,0);
  if (trajectory_options_.use_visual_slam_pose) {
    subscribers_[map_index].push_back(
        {SubscribeWithHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(
             &Node::HandleVisualSlamPoseMessage, map_index, kVisualSlamPose,
             node_handle_, this),
         kVisualSlamPose});
  }
  RCLCPP_INFO(node_handle_->get_logger(), "subscribe vslam pose successfully.");
  last_stamp_set_map_ = node_handle_->now().seconds();

  initFinish_ = false;
  setMapWorking_ = false;
  LOG(INFO) << "cartographer: end set occMap";
  return result;
}

bool Node::checkInitPose(const geometry_msgs::msg::Pose& msg) {
  float resolution = occ_map_->info.resolution;
  unsigned int width = occ_map_->info.width;
  unsigned int height = occ_map_->info.height;

  Eigen::Vector2d map_origin{occ_map_->info.origin.position.x, occ_map_->info.origin.position.y};
  Eigen::Vector2d map_max = map_origin + Eigen::Vector2d{static_cast<double>(width*resolution), static_cast<double>(height*resolution)};

  if ( msg.position.x < map_origin[0] || msg.position.y < map_origin[1]
    || msg.position.x > map_max[0]    || msg.position.y > map_max[1] ) {

    LOG(ERROR) << "received init position (" << msg.position.x << ", " 
               << msg.position.y << ")"  << " out of map.";
    return false;
  }

  Eigen::Vector2d pose_to_origin(msg.position.x - map_origin[0], msg.position.y - map_origin[1]);
  Eigen::Vector2i pose_in_occmap = (pose_to_origin / resolution).cast<int>();

  unsigned int pose_index = pose_in_occmap[0] + pose_in_occmap[1] * width;
  int occupancy_value = occ_map_->data[pose_index];

  if (occupancy_value != 0) {
    LOG(ERROR) << "received init position (" << msg.position.x << ", " 
               << msg.position.y << ")"  << " is not free.";
    return false;
  }

  return true;
}

void Node::setInitialOptions(const cartographer::transform::Rigid3d& pose) {
  
  while(setMapWorking_ && rclcpp::ok()){
    LOG(WARNING) << "setInitialOptions: set occMap is working!";
    rclcpp::Rate(10.).sleep(); // hz
    // using namespace std::chrono_literals;
    // std::this_thread::sleep_for(100ms);
  }
  ::cartographer::mapping::proto::InitialTrajectoryPose
      initial_trajectory_pose;
  initial_trajectory_pose.set_to_trajectory_id(0);
  *initial_trajectory_pose.mutable_relative_pose() =
      cartographer::transform::ToProto(pose);
  initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
      ::cartographer_ros::FromRos(rclcpp::Time(0.))));
  *trajectory_options_.trajectory_builder_options
        .mutable_initial_trajectory_pose() = initial_trajectory_pose;

  if(map_builder_bridge_->GetTrajectoryNum() > 1) { // || initFinish_
    LOG(INFO) << "trajectory number more than one when set initial pose, reset occ map.";
    SetOccMap(*occ_map_);
  }
  StartTrajectoryWithDefaultTopics(trajectory_options_);
  publish_local_trajectory_data_timer_->reset();
  initFinish_ = true;
  globalMessagePtr_->setInitState(cartographer::InitState::BUILD_CONSTRAINT);
  LOG(INFO) << "setInitialOptions: set initial pose finish!";

}

int Node::SetInitialPose(const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& init_poses) {
  using SetInitPoseResult = sps_common_msgs::srv::AidedPose::Response;
  CHECK_GT(init_poses.size(),0);
  absl::MutexLock lock(&setInitialPoseMutex_);
  int last_init_state = globalMessagePtr_->getInitState();
  if (last_init_state == cartographer::InitState::SET_INITIAL_POSE) {
    LOG(ERROR) << "last set init pose has not finished, refused...";
    return SetInitPoseResult::SETTING_POSE;
  }
  RCLCPP_INFO(node_handle_->get_logger(), "cartographer node:set initial pose start");
  globalMessagePtr_->setInitState(cartographer::InitState::SET_INITIAL_POSE);
  if (init_poses.size() > 1) { // multi initial poses case
    LOG(INFO) << "Received multi poses for initialization.";
    std::vector<carto::transform::Rigid2d> carto_poses;
    for (const auto pose : init_poses) {
      auto carto_pose = ToRigid2d(pose.pose.pose);
      LOG(INFO) << "multi pose: " << carto_pose;
      carto_poses.push_back(carto_pose);
    }
    globalMessagePtr_->setMultiInitPoses(carto_poses);
  }
  auto msg = init_poses[0]; // use the first one to initialize trajectory(when multi initial poses)
  const auto pose = ToRigid3d(msg.pose.pose);
  LOG(INFO) << "init trajectory with pose x: " << pose.translation()[0] << " y: " << pose.translation()[1] << " yaw: " << GetYaw(pose) * 180.0 / 3.1415;
  if (!pose.IsValid()) {
    LOG(ERROR) <<  "Invalid pose argument. Orientation quaternion must be normalized.";
    // return SetInitPoseResult::POSE_UNVALID;
  }

  if(!checkInitPose(msg.pose.pose)) {
    RCLCPP_ERROR(node_handle_->get_logger(), "cartographer node: check init pose illegal.");
    globalMessagePtr_->setInitState(last_init_state);
    return SetInitPoseResult::POSE_UNVALID;
  }

  globalMessagePtr_->setFlagInsertSubMapID1(false);
  globalMessagePtr_->setFlagInitConstraint(false);
  last_stamp_init_pose_ = node_handle_->now().seconds();

  if (setMapWorking_) { // async set some init options when setting map
    LOG(WARNING) << "set occmap is working, create thread to set initial options!";
    std::thread t1(&Node::setInitialOptions,this,std::move(pose));
    t1.detach();
    return SetInitPoseResult::SUCCESS;
  } else {
    setInitialOptions(std::move(pose));
    return SetInitPoseResult::SUCCESS;
  }

}

// bool Node::SetInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped& msg){
//   absl::MutexLock lock(&setInitialPoseMutex_);
  
//   if(!checkInitPose(msg.pose.pose)) {
//     RCLCPP_ERROR(node_handle_->get_logger(), "cartographer node: check init pose illegal.");
//     return false;
//   }

//   globalMessagePtr_->setInitState(cartographer::InitState::SET_INITIAL_POSE);
//   RCLCPP_INFO(node_handle_->get_logger(), "cartographer node:set initial pose start");
//   globalMessagePtr_->setFlagInsertSubMapID1(false);
//   last_stamp_init_pose_ = node_handle_->now().seconds();
//   int waitMap = 0;
//   while(setMapWorking_ && rclcpp::ok()){
//     LOG(WARNING) << "cartographer: set occMap is working!";
//     rclcpp::Rate(10.).sleep(); // hz
//     // using namespace std::chrono_literals;
//     // std::this_thread::sleep_for(100ms);
//     if(waitMap > 70){
//       LOG(ERROR) << "cartographer: map not ready, set initialPose failed!";
//       return false;
//     }
//     waitMap++;
//   }
//   const auto pose = ToRigid3d(msg.pose.pose);
//   LOG(INFO) <<  "int pose x: " << pose.translation()[0] << " y: " << pose.translation()[1] << " yaw: " << GetYaw(pose) * 180.0 / 3.1415;
//   if (!pose.IsValid()) {
//     LOG(ERROR) <<  "Invalid pose argument. Orientation quaternion must be normalized.";
//   }
//   ::cartographer::mapping::proto::InitialTrajectoryPose
//       initial_trajectory_pose;
//   initial_trajectory_pose.set_to_trajectory_id(0);
//   *initial_trajectory_pose.mutable_relative_pose() =
//       cartographer::transform::ToProto(pose);
//   initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
//       ::cartographer_ros::FromRos(rclcpp::Time(0.))));
//   *trajectory_options_.trajectory_builder_options
//         .mutable_initial_trajectory_pose() = initial_trajectory_pose;

//   if(map_builder_bridge_->GetTrajectoryNum() > 1) { // || initFinish_
//     LOG(INFO) << "trajectory number more than one when set initial pose, reset occ map.";
//     SetOccMap(*occ_map_);
//   }
//   StartTrajectoryWithDefaultTopics(trajectory_options_);
//   publish_local_trajectory_data_timer_->reset();
//   initFinish_ = true;
//   LOG(INFO) << "cartographer node:set initial pose finish!";
//   globalMessagePtr_->setInitState(cartographer::InitState::MATCH_WITH_MAP);
//   return true;
// }

bool Node::SetFlagMatchMap(const bool& msg){
  globalMessagePtr_->setFlagMatchMap(msg);
  return true;
}

bool Node::SetRelocalizationState(const int& reloc_state){
  if(reloc_state == 0){
      return globalMessagePtr_->setFlagTrigGlobalLocalization(false);
  }
  else{
    return  globalMessagePtr_->setFlagTrigGlobalLocalization(true);
  }
}

bool Node::setFlagIsLocalization(const bool& flagIsLocalization){
    globalMessagePtr_->setFlagIsLocalization(flagIsLocalization);
    return true;
}
bool Node::naviTypeCB(const int& naviType){
  if(naviType == 1 || naviType == 3){
    globalMessagePtr_->setFlagMatchMap(false);
  }
  else{
    globalMessagePtr_->setFlagMatchMap(true);
  }
  return true;
}

double Node::GetLocConfidence() {
  if(map_builder_bridge_->getScorePoseFromMapPtr() == nullptr){
    RCLCPP_WARN_THROTTLE(node_handle_->get_logger(), *(node_handle_->get_clock()),5000, "Node:: map_builder_bridge_->scorePoseFromMapPtr_ is nullptr");
    return 0.0;
  }
  return *(map_builder_bridge_->getScorePoseFromMapPtr());
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const TrajectoryOptions& options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, kImuTopic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(SensorId{SensorType::ODOMETRY, kOdometryTopic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, kNavSatFixTopic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, kLandmarkTopic});
  }
  return expected_topics;
}

int Node::AddTrajectory(const TrajectoryOptions& options) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, trajectory_id);
  wall_timers_.push_back(node_handle_->create_wall_timer(
      std::chrono::milliseconds(int(kTopicMismatchCheckDelaySec * 1000)),
      [this]() {
        MaybeWarnAboutTopicMismatch();
      }));
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void Node::LaunchSubscribers(const TrajectoryOptions& options,
                             const int trajectory_id) {
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kLaserScanTopic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, node_handle_, this),
         topic});
  }
  for (const std::string& topic : ComputeRepeatedTopicNames(
           kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic, node_handle_, this),
         topic});
  }
  for (const std::string& topic :
       ComputeRepeatedTopicNames(kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic, node_handle_, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, kImuTopic,
                                                node_handle_, this),
         kImuTopic});
  }

  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, kOdometryTopic,
                                                  node_handle_, this),
         kOdometryTopic});
  }
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, kNavSatFixTopic,
             node_handle_, this),
         kNavSatFixTopic});
  }
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, kLandmarkTopic,
             node_handle_, this),
         kLandmarkTopic});
  }
  // if (options.use_visual_slam_pose) {
  //   subscribers_[trajectory_id].push_back(
  //       {SubscribeWithHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(
  //            &Node::HandleVisualSlamPoseMessage, trajectory_id, kVisualSlamPose,
  //            node_handle_, this),
  //        kVisualSlamPose});
  // }
  if (options.use_tag_pose) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<geometry_msgs::msg::PoseWithCovarianceStamped>(
             &Node::HandleTagPoseMessage, trajectory_id, kTagPose,
             node_handle_, this),
         kTagPose});
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<geometry_msgs::msg::PoseStamped>(
             &Node::HandleTagToRobotMessage, trajectory_id, kTagToRobot,
             node_handle_, this),
         kTagToRobot});    
  }

  use_confidence_ = options.use_confidence;
  laserTh_ = options.laserTh;
  visualTh_ = options.visualTh;
  cancelTh_ = options.cancelTh;
  // subscribers_[trajectory_id].push_back(
  //     {SubscribeWithHandler<laser_line_extraction::LineSegmentList>(
  //           &Node::HandleLineExtractionMessage, trajectory_id, kLineExtraction,
  //           &node_handle_, this),
  //       kLineExtraction});  

}

bool Node::ValidateTrajectoryOptions(const TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(const TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::msg::StatusResponse Node::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
  cartographer_ros_msgs::msg::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message = "Trajectory " + std::to_string(trajectory_id) + " doesn't exist.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message = "Trajectory " + std::to_string(trajectory_id) + " is in '" +
    TrajectoryStateToString(it->second) + "' state.";
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::msg::StatusCode::OK
          : cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  return status_response;
}

cartographer_ros_msgs::msg::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  std::cout <<"finish under lock 1\n";
  cartographer_ros_msgs::msg::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    std::cout <<"finish under lock 2\n";
    status_response.message = "Trajectory " + std::to_string(trajectory_id) + " already pending to finish.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }
  std::cout <<"finish under lock 3\n";

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }
  std::cout <<"finish under lock 4\n";
  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
	    std::cout <<"finish under lock 4-1 " << entry.subscriber->get_topic_name() << std::endl;
      // while(!handleOdomFinish_){
      //   RCLCPP_INFO(node_handle_->get_logger(), "wait for handleOdomFinish !!!");
      // };
      RCLCPP_INFO(node_handle_->get_logger(), "wait for handleOdomFinish1 !!!");
      rclcpp::Rate r(1.0); // hz
      r.sleep();
      RCLCPP_INFO(node_handle_->get_logger(), "wait for handleOdomFinish2 !!!");
      // rclcpp::Rate(1.0).sleep(); // hz
      // RCLCPP_INFO(node_handle_->get_logger(), "wait for handleOdomFinish3 !!!");
      // entry.subscriber.shutdown();
      std::cout <<"finish under lock 4-2\n";
      subscribed_topics_.erase(entry.topic);
      std::cout <<"finish under lock 4-3\n";
      std::cout << "Shutdown the subscriber of [" << entry.topic << "]\n";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  std::cout <<"finish under lock 5\n";
  map_builder_bridge_->FinishTrajectory(trajectory_id);
  std::cout <<"finish under lock 6\n";
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  std::cout <<"finish under lock 7\n";
  status_response.message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
  std::cout <<"finish under lock 8\n";
  return status_response;
}

bool Node::HandleStartTrajectory(
    const cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response) {
  TrajectoryOptions trajectory_options;
  std::tie(std::ignore, trajectory_options) = LoadOptions(
      request->configuration_directory, request->configuration_basename);

  if (request->use_initial_pose) {
    const auto pose = ToRigid3d(request->initial_pose);
    if (!pose.IsValid()) {
      response->status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response->status.message;
      response->status.code =
          cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
      return true;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    response->status = TrajectoryStateToStatus(
        request->relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response->status.message;
      return true;
    }

    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(
        request->relative_to_trajectory_id);
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(rclcpp::Time(0))));
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response->status.message = "Invalid trajectory options.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) {
    response->status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else {
    response->status.message = "Success.";
    response->trajectory_id = AddTrajectory(trajectory_options);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  }
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  CHECK(ValidateTrajectoryOptions(options));
  AddTrajectory(options);
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

bool Node::HandleGetTrajectoryStates(
    const cartographer_ros_msgs::srv::GetTrajectoryStates::Request::SharedPtr ,
    cartographer_ros_msgs::srv::GetTrajectoryStates::Response::SharedPtr response) {
  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&mutex_);
  response->status.code = ::cartographer_ros_msgs::msg::StatusCode::OK;
  response->trajectory_states.header.stamp = node_handle_->now();
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    response->trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::DELETED);
        break;
    }
  }
  return true;
}

bool Node::HandleFinishTrajectory(
    const cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = FinishTrajectoryUnderLock(request->trajectory_id);
  return true;
}

bool Node::HandleWriteState(
    const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
    cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  if (map_builder_bridge_->SerializeState(request->filename,
                                         request->include_unfinished_submaps)) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message =
        "State written to '" + request->filename + "'.";
  } else {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message =
        "Failed to write '" + request->filename + "'.";
  }
  return true;
}

bool Node::HandleReadMetrics(
    const cartographer_ros_msgs::srv::ReadMetrics::Request::SharedPtr,
    cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->timestamp = node_handle_->now();
  if (!metrics_registry_) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::UNAVAILABLE;
    response->status.message = "Collection of runtime metrics is not activated.";
    return true;
  }
  metrics_registry_->ReadMetrics(response);
  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  response->status.message = "Successfully read metrics.";
  return true;
}

void Node::FinishAllTrajectories() {
  RCLCPP_INFO(node_handle_->get_logger(), "Carto: finishi all traje 1");
  absl::MutexLock lock(&mutex_);
  RCLCPP_INFO(node_handle_->get_logger(), "Carto: finishi all traje 2");
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    RCLCPP_INFO(node_handle_->get_logger(), "Carto: finishi all traje 3");
    if (entry.second == TrajectoryState::ACTIVE) {
      RCLCPP_INFO(node_handle_->get_logger(), "Carto: finishi all traje 4");
      const int trajectory_id = entry.first;
      RCLCPP_INFO(node_handle_->get_logger(), "Carto: finishi all traje 5");
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::msg::StatusCode::OK);
      RCLCPP_INFO(node_handle_->get_logger(), "Carto: finishi all traje 6");
    }
  }
  RCLCPP_INFO(node_handle_->get_logger(), "Carto: finishi all traje 7");
}

bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::msg::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_->RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  last_stamp_odom_ = node_handle_->now().seconds();//ros::Time::now().toSec();
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}
void Node::HandleVisualSlamPoseMessage(
    const int trajectory_id, const std::string& sensor_id,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  int trajectory_num = map_builder_bridge_->GetTrajectoryNum();
  if (trajectory_num == 1 ) { // not initialized 
    double time_now = node_handle_->now().seconds();
    double set_map_duration = time_now - last_stamp_set_map_;
    double set_pose_duration = time_now - last_stamp_init_pose_;
    double score = msg->pose.covariance[0];
    if (!setMapWorking_ &&  globalMessagePtr_->getInitState() == cartographer::InitState::NOT_INITIALZED && //set_pose_duration > 5.0 &&
        set_map_duration > 10.0 && score > 0.7) { // for mapping finished but not set pose
      LOG(INFO) << "Use VSLAM pose for Initialization";
      std::vector<geometry_msgs::msg::PoseWithCovarianceStamped> init_pose;
      init_pose.emplace_back(*msg);
      std::thread t1{&Node::SetInitialPose,this,std::move(init_pose)};
      // std::thread t1{&Node::SetInitialPose,this,std::move(*msg)};
      t1.detach();
      return;
    }
  }

  if(node_handle_->now().seconds() - last_stamp_init_pose_ < 5.0){
      LOG(WARNING) << "vslam stamp is too near to init, last intit pose stamp: " << last_stamp_init_pose_ << " now: " << node_handle_->now().seconds();
  }
  else if(msg != nullptr &&  (node_handle_->now() - msg->header.stamp).seconds() < 0.5){
    Eigen::Vector3d translation(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
    Eigen::Quaternion<double> angle((msg->pose.pose.orientation.w),
                                    (msg->pose.pose.orientation.x),
                                    (msg->pose.pose.orientation.y),
                                    (msg->pose.pose.orientation.z));
    carto::transform::Rigid3d pose(translation,angle);
    // LOG(INFO) << "visual pose: x: " << translation[0] << " y: " << translation[1] << " yaw: " << GetYaw(pose) * 180.0 / 3.1415
    //           << " score: " << msg->pose.covariance[0];
    double score = msg->pose.covariance[0];
    vslam_score_ = score;
    globalMessagePtr_->setPoseFromVisualSlam({carto::transform::Project2D(pose), score});
  }
  else{
    LOG(WARNING) << "vslam pose is too old, now stamp: " << node_handle_->now().seconds() 
                 << " vslam pose stamp: " <<  msg->header.stamp.sec
                 << " diff: " << (node_handle_->now() - msg->header.stamp).seconds(); 
  }
}

void Node::HandleTagPoseMessage(
    const int trajectory_id, const std::string& sensor_id,
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if(msg != nullptr){
    Eigen::Vector3d translation(msg->pose.pose.position.x,
                                msg->pose.pose.position.y,
                                msg->pose.pose.position.z);
    Eigen::Quaternion<double> angle((msg->pose.pose.orientation.w),
                                    (msg->pose.pose.orientation.x),
                                    (msg->pose.pose.orientation.y),
                                    (msg->pose.pose.orientation.z));
    carto::transform::Rigid3d pose(translation,angle);
    LOG(INFO) << "tag pose : " << pose;
    LOG(INFO) << "tag pose.yaw: " << GetYaw(pose) * 180.0 / 3.1415;
    double score = msg->pose.covariance[0];
    globalMessagePtr_->setPoseFromTag({carto::transform::Project2D(pose), score});
  }
}

void Node::HandleTagToRobotMessage(
    const int trajectory_id, const std::string& sensor_id,
    const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if(msg != nullptr){
    LOG(INFO) << "Node::HandleTagToRobotMessage";
    map_builder_bridge_->sensor_bridge(trajectory_id)
        ->HandleTagToRobotMessage(sensor_id, msg);
  }
}

// void Node::HandleLineExtractionMessage(
//       int trajectory_id, const std::string& sensor_id,
//       const laser_line_extraction::LineSegmentList::ConstPtr& msg) {
//   absl::MutexLock lock(&mutex_);
//   if(msg != nullptr){
//     laser_line_extraction::LineSegmentList lineSegmentList = *msg;
//     float angleMin = 3.14;
//     float angleMax = -3.14;
//     if(lineSegmentList.line_segments.size() < 2){
//       globalMessagePtr_->setFlagHasSufficientFeatures(false);
//       return;
//     }

//     for(auto lineSegment : lineSegmentList.line_segments){
//       if(lineSegment.angle < 0.0f){
//         lineSegment.angle += 3.14159265; 
//       }
//       if(lineSegment.angle < angleMin){
//         angleMin = lineSegment.angle;
//       }
//       if(lineSegment.angle > angleMax){
//         angleMax = lineSegment.angle;
//       }
//     }
//     globalMessagePtr_->setFlagHasSufficientFeatures(angleMax - angleMin > 0.35);
//   }
// }
void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  last_stamp_laser_ = node_handle_->now().seconds();//ros::Time::now().toSec();
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }

  globalMessagePtr_->setMaxRange(msg->range_max);
  
  // 判断激光有效数据占比
  int count=0;   
  std::vector<float> ranges = msg->ranges;
  double range_size = ranges.size();
  for(std::vector<float>::iterator it = ranges.begin();it !=ranges.end();it++)
  {
      if(std::isnan(*it)|| std::isinf(*it))
      {
          count++;
      }
  }
  double rate = (range_size-count)/range_size*100;
  if(rate < 20.0)
  {
    bad_scan_flag_ = true;
  }
  else
  {
    bad_scan_flag_ = false;
  }

  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_->SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->LoadState(state_filename, load_frozen_state);
}
void Node::MaybeWarnAboutTopicMismatch() {
/*
  ::ros::master::V_TopicInfo ros_topics;
  ::ros::master::getTopics(ros_topics);
  std::set<std::string> published_topics;
  std::stringstream published_topics_string;
  for (const auto& it : ros_topics) {
    std::string resolved_topic = node_handle_.resolveName(it.name, false);
    published_topics.insert(resolved_topic);
    published_topics_string << resolved_topic << ",";
  }
  bool print_topics = false;
  for (const auto& entry : subscribers_) {
    int trajectory_id = entry.first;
    for (const auto& subscriber : entry.second) {
      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
      if (published_topics.count(resolved_topic) == 0) {
        LOG(WARNING) << "Expected topic \"" << subscriber.topic
                     << "\" (trajectory " << trajectory_id << ")"
                     << " (resolved topic \"" << resolved_topic << "\")"
                     << " but no publisher is currently active.";
        print_topics = true;
      }
    }
  }
*/
  // if (print_topics) {
  //   LOG(WARNING) << "Currently available topics are: "
  //                << published_topics_string.str();
  // }
}

}  // namespace cartographer_ros

