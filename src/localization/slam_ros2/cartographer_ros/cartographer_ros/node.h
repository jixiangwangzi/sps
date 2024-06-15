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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <thread>

#include "absl/synchronization/mutex.h"
#include "cartographer/common/fixed_ratio_sampler.h"
#include "cartographer/common/global_message.h"
#include "cartographer/mapping/map_builder_interface.h"
#include "cartographer/mapping/pose_extrapolator.h"
#include "cartographer_ros/map_builder_bridge.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/node_constants.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/trajectory_options.h"
#include "cartographer_ros_msgs/srv/finish_trajectory.hpp"
#include "cartographer_ros_msgs/srv/get_trajectory_states.hpp"
#include "cartographer_ros_msgs/srv/read_metrics.hpp"
#include "cartographer_ros_msgs/srv/start_trajectory.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "cartographer_ros_msgs/msg/submap_entry.hpp"
#include "cartographer_ros_msgs/msg/submap_list.hpp"
#include "cartographer_ros_msgs/srv/submap_query.hpp"
#include "cartographer_ros_msgs/srv/write_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "tf2_ros/transform_broadcaster.h"
// #include <opencv2/opencv.hpp>
#include <tf2/utils.h>
#include "cartographer_ros/trajectory_options.h"
#include "std_msgs/msg/int32.hpp"
// #include <eigen_conversions/eigen_msg.h>
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "laser_line_extraction/LineSegmentList.h"
#include "sps_common_msgs/srv/aided_pose.hpp"
#include "sps_common_msgs/msg/initial_slam_state.hpp"
DECLARE_bool(collect_metrics);
DECLARE_string(configuration_cloud_directory);
DECLARE_string(configuration_cloud_basename);
DECLARE_string(configuration_directory);
DECLARE_string(configuration_basename);
DECLARE_string(load_state_filename);
DECLARE_bool(load_frozen_state);
DECLARE_bool(start_trajectory_with_default_topics);
DECLARE_string(save_state_filename);
    
DECLARE_double(resolution);
DECLARE_double(publish_period_sec);

namespace cartographer_ros {
  struct Subscriber {
    rclcpp::SubscriptionBase::SharedPtr subscriber;

    // ::ros::Subscriber::getTopic() does not necessarily return the same
    // std::string
    // it was given in its constructor. Since we rely on the topic name as the
    // unique identifier of a subscriber, we remember it ourselves.
    std::string topic;
  };
// Wires up ROS topics to SLAM.
class Node {
 public:
  /*Node(const NodeOptions& node_options,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       std::shared_ptr<tf2_ros::Buffer> tf_buffer, bool collect_metrics);
  Node(const NodeOptions& node_options, rclcpp::Node::SharedPtr node_handle,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       std::shared_ptr<tf2_ros::Buffer> tf_buffer, bool collect_metrics);*/
  Node(const NodeOptions& node_options, rclcpp::Node::SharedPtr node_handle,
       std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
       tf2_ros::Buffer* const tf_buffer, const bool collect_metrics, TrajectoryOptions& trajectory_options);
  ~Node();

  Node(const Node&) = delete;
  Node& operator=(const Node&) = delete;

  // Finishes all yet active trajectories.
  void FinishAllTrajectories();
  // Finishes a single given trajectory. Returns false if the trajectory did not
  // exist or was already finished.
  bool FinishTrajectory(int trajectory_id);

  // Runs final optimization. All trajectories have to be finished when calling.
  void RunFinalOptimization();

  // Starts the first trajectory with the default topics.
  void StartTrajectoryWithDefaultTopics(const TrajectoryOptions& options);

  // Returns unique SensorIds for multiple input bag files based on
  // their TrajectoryOptions.
  // 'SensorId::id' is the expected ROS topic name.
  std::vector<
      std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
  ComputeDefaultSensorIdsForMultipleBags(
      const std::vector<TrajectoryOptions>& bags_options) const;

  // Adds a trajectory for offline processing, i.e. not listening to topics.
  int AddOfflineTrajectory(
      const std::set<
          cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
          expected_sensor_ids,
      const TrajectoryOptions& options);

  // The following functions handle adding sensor data to a trajectory.
  void HandleOdometryMessage(int trajectory_id, const std::string& sensor_id,
                             const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
  void HandleNavSatFixMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg);
  void HandleLandmarkMessage(
      int trajectory_id, const std::string& sensor_id,
      const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg);
  void HandleVisualSlamPoseMessage(
      int trajectory_id, const std::string& sensor_id,
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
  void HandleTagPoseMessage(
      int trajectory_id, const std::string& sensor_id,
      const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr& msg);
  void HandleTagToRobotMessage(
      int trajectory_id, const std::string& sensor_id,
      const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
//   void HandleLineExtractionMessage(
//       int trajectory_id, const std::string& sensor_id,
//       const laser_line_extraction::LineSegmentList::ConstPtr& msg);
  void HandleImuMessage(int trajectory_id, const std::string& sensor_id,
                        const sensor_msgs::msg::Imu::ConstSharedPtr &msg);
  void HandleLaserScanMessage(int trajectory_id, const std::string& sensor_id,
                              const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg);
  void HandleMultiEchoLaserScanMessage(
      int trajectory_id, const std::string& sensor_id,
      const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg);
  void HandlePointCloud2Message(int trajectory_id, const std::string& sensor_id,
                                const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

  // Serializes the complete Node state.
  void SerializeState(const std::string& filename,
                      const bool include_unfinished_submaps);

  // Loads a serialized SLAM state from a .pbstream file.
  void LoadState(const std::string& state_filename, bool load_frozen_state);

  rclcpp::Node::SharedPtr node_handle();
//   int loadMap(const std::string &path);
  bool SetOccMap(const nav_msgs::msg::OccupancyGrid& occ_map);
  int SetInitialPose(const std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>& init_poses);
  bool SetFlagMatchMap(const bool& msg);
  bool SetRelocalizationState(const int& reloc_state_msg);
  bool naviTypeCB(const int& naviType);
  void setInitFinish(bool initFinish){ initFinish_ = initFinish;}
  bool getInitFinish(){return initFinish_;};
  // bool getInitFinish(){return globalMessagePtr_->getInitState() == cartographer::InitState::SUCCESS;};
  bool setFlagIsLocalization(const bool& flagIsLocalization);
  bool checkInitPose(const geometry_msgs::msg::Pose& msg);

  void setInitialOptions(const cartographer::transform::Rigid3d& pose);

  bool reset();
  
  bool GetFlagInWall(){return globalMessagePtr_->getFlagInWall();};
  TrajectoryOptions getTrajectory_options(){return trajectory_options_;};
  std::unordered_map<int, std::vector<Subscriber>> GetSubscribers(){return subscribers_;};
  double GetLocConfidence();

  double GetVisualConfidence() {return vslam_score_; }
  bool  GetJumpFlag() { return pose_jump_flag_ ;}
  bool  CheckSwitchMapFinish() {return map_builder_bridge_->CheckSwitchMapFinish();}
  bool  IsBadScanData() {return bad_scan_flag_;}   
  bool  UseVisualFlag() { return globalMessagePtr_->getFlagUseVisualSlamPose();}
  bool  UseConfidence() {return use_confidence_;}
  bool  InsertSubMapID1() {return globalMessagePtr_->getFlagInsertSubMapID1();}
  bool  GetLaserLostFlag() {return laser_lost_;}
  bool  GetOdomLostFlag() {return odom_lost_;}
  
  double GetLaserTh() { return laserTh_;}
  double GetVisualTh() { return visualTh_;}
  double GetCancelTh() {return cancelTh_;}
 private:
//   struct Subscriber {
//     ::ros::Subscriber subscriber;

//     // ::ros::Subscriber::getTopic() does not necessarily return the same
//     // std::string
//     // it was given in its constructor. Since we rely on the topic name as the
//     // unique identifier of a subscriber, we remember it ourselves.
//     std::string topic;
//   };

  bool HandleSubmapQuery(
      const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
      cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response);
  bool HandleTrajectoryQuery(
      const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
      cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response);
  bool HandleStartTrajectory(
      const cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
      cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response);
  bool HandleFinishTrajectory(
      const cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
      cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response);
  bool HandleWriteState(
      const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
      cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response);
  bool HandleGetTrajectoryStates(
      const cartographer_ros_msgs::srv::GetTrajectoryStates::Request::SharedPtr,
      cartographer_ros_msgs::srv::GetTrajectoryStates::Response::SharedPtr response);
  bool HandleReadMetrics(
      const cartographer_ros_msgs::srv::ReadMetrics::Request::SharedPtr,
      cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response);

  // Returns the set of SensorIds expected for a trajectory.
  // 'SensorId::id' is the expected ROS topic name.
  std::set<::cartographer::mapping::TrajectoryBuilderInterface::SensorId>
  ComputeExpectedSensorIds(const TrajectoryOptions& options) const;
  int AddTrajectory(const TrajectoryOptions& options);
  void LaunchSubscribers(const TrajectoryOptions& options, int trajectory_id);
  void PublishSubmapList();
  void AddExtrapolator(int trajectory_id, const TrajectoryOptions& options);
  void AddSensorSamplers(int trajectory_id, const TrajectoryOptions& options);
  void PublishLocalTrajectoryData();
  void PublishTrajectoryNodeList();
  void PublishLandmarkPosesList();
  void PublishConstraintList();
  bool ValidateTrajectoryOptions(const TrajectoryOptions& options);
  bool ValidateTopicNames(const TrajectoryOptions& options);
  void PublishOccMap(/*const ::ros::WallTimerEvent& timer_event*/);
  cartographer_ros_msgs::msg::StatusResponse FinishTrajectoryUnderLock(
      int trajectory_id) EXCLUSIVE_LOCKS_REQUIRED(mutex_);
  void MaybeWarnAboutTopicMismatch(/*const ::ros::WallTimerEvent&*/);

  void PubLocInitFinish(/*const ::ros::WallTimerEvent& timer_event*/);

  // Helper function for service handlers that need to check trajectory states.
  cartographer_ros_msgs::msg::StatusResponse TrajectoryStateToStatus(
      int trajectory_id,
      const std::set<
          cartographer::mapping::PoseGraphInterface::TrajectoryState>&
          valid_states);
  const NodeOptions node_options_;
  rclcpp::Node::SharedPtr node_handle_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  absl::Mutex mutex_;
  absl::Mutex setOccMapMutex_;
  absl::Mutex setInitialPoseMutex_;
  std::unique_ptr<cartographer_ros::metrics::FamilyFactory> metrics_registry_;
  std::shared_ptr<::cartographer::GlobalMessage> globalMessagePtr_;
//   MapBuilderBridge map_builder_bridge_ GUARDED_BY(mutex_);
  std::shared_ptr<MapBuilderBridge> map_builder_bridge_ GUARDED_BY(mutex_);

  ::rclcpp::Publisher<::cartographer_ros_msgs::msg::SubmapList>::SharedPtr submap_list_publisher_;
  ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr trajectory_node_list_publisher_;
  ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr landmark_poses_list_publisher_;
  ::rclcpp::Publisher<::visualization_msgs::msg::MarkerArray>::SharedPtr constraint_list_publisher_;
  ::rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tracked_pose_publisher_;
  ::rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_matched_point_cloud_publisher_;
  ::rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr submap_point_cloud_publisher_;
  ::rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_publisher_;
  ::rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_loc_init_finish_;
  ::rclcpp::Publisher<sps_common_msgs::msg::InitialSlamState>::SharedPtr initial_state_publisher_;
  // These ros::ServiceServers need to live for the lifetime of the node.
//   std::vector<::ros::ServiceServer> service_servers_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::SubmapQuery>::SharedPtr submap_query_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::TrajectoryQuery>::SharedPtr trajectory_query_server;
  ::rclcpp::Service<cartographer_ros_msgs::srv::StartTrajectory>::SharedPtr start_trajectory_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::FinishTrajectory>::SharedPtr finish_trajectory_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::WriteState>::SharedPtr write_state_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::GetTrajectoryStates>::SharedPtr get_trajectory_states_server_;
  ::rclcpp::Service<cartographer_ros_msgs::srv::ReadMetrics>::SharedPtr read_metrics_server_;
  ::rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  bool initFinish_;
  bool setMapWorking_;
  TrajectoryOptions trajectory_options_;

  struct TrajectorySensorSamplers {
    TrajectorySensorSamplers(const double rangefinder_sampling_ratio,
                             const double odometry_sampling_ratio,
                             const double fixed_frame_pose_sampling_ratio,
                             const double imu_sampling_ratio,
                             const double landmark_sampling_ratio)
        : rangefinder_sampler(rangefinder_sampling_ratio),
          odometry_sampler(odometry_sampling_ratio),
          fixed_frame_pose_sampler(fixed_frame_pose_sampling_ratio),
          imu_sampler(imu_sampling_ratio),
          landmark_sampler(landmark_sampling_ratio) {}

    ::cartographer::common::FixedRatioSampler rangefinder_sampler;
    ::cartographer::common::FixedRatioSampler odometry_sampler;
    ::cartographer::common::FixedRatioSampler fixed_frame_pose_sampler;
    ::cartographer::common::FixedRatioSampler imu_sampler;
    ::cartographer::common::FixedRatioSampler landmark_sampler;
  };

  // These are keyed with 'trajectory_id'.
  std::map<int, ::cartographer::mapping::PoseExtrapolator> extrapolators_;
  std::map<int, builtin_interfaces::msg::Time> last_published_tf_stamps_;
  std::unordered_map<int, TrajectorySensorSamplers> sensor_samplers_;
  std::unordered_map<int, std::vector<Subscriber>> subscribers_;
  std::unordered_set<std::string> subscribed_topics_;
  std::unordered_set<int> trajectories_scheduled_for_finish_;
  std::shared_ptr<nav_msgs::msg::OccupancyGrid> occ_map_;
  // We have to keep the timer handles of ::ros::WallTimers around, otherwise
  // they do not fire.
  std::vector<::rclcpp::TimerBase::SharedPtr> wall_timers_;

  // The timer for publishing local trajectory data (i.e. pose transforms and
  // range data point clouds) is a regular timer which is not triggered when
  // simulation time is standing still. This prevents overflowing the transform
  // listener buffer by publishing the same transforms over and over again.
  ::rclcpp::TimerBase::SharedPtr publish_local_trajectory_data_timer_;
//   ::rclcpp::TimerBase::SharedPtr submap_list_timer_;
//   ::rclcpp::TimerBase::SharedPtr trajectory_node_list_timer_;
//   ::rclcpp::TimerBase::SharedPtr landmark_pose_list_timer_;
//   ::rclcpp::TimerBase::SharedPtr constrain_list_timer_;
//   ::rclcpp::TimerBase::SharedPtr occ_map_timer_;
//   ::rclcpp::TimerBase::SharedPtr loc_init_finish_timer_;

  geometry_msgs::msg::TransformStamped last_stamped_transform_;
  cartographer::transform::Rigid3d last_tracking_to_map_;
  double last_stamp_odom_; //最后一次收到odom数据的时间戳
  double last_stamp_laser_; //最后一次收激光数据的时间戳
  double last_stamp_init_pose_ = 0.0;  //最后一次收到初始位姿的时间戳
  double last_stamp_set_map_ = 0.0;
  Eigen::Matrix<double, 2, 1> translation_map_to_origin_;


  double vslam_score_ = 0.0;                             //视觉位姿得分
  bool  pose_jump_flag_ = false;                         //位姿是否跳变(超过0.8m)
  bool  bad_scan_flag_ = false;                          //激光有效点的数量小于20%
  bool  use_confidence_ = false;                         //是否使用得分判断
  bool  laser_lost_ = false;
  bool  odom_lost_ = false;
  double laserTh_=0.0;
  double visualTh_=0.0;
  double cancelTh_=0.0;

};

}  // namespace cartographer_ros
#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_H
