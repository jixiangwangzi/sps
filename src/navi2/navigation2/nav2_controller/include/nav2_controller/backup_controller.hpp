// Copyright (c) 2020 Shrijit Singh
// Copyright (c) 2020 Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_
#define NAV2_REGULATED_PURE_PURSUIT_CONTROLLER__REGULATED_PURE_PURSUIT_CONTROLLER_HPP_

#include <string>
#include <vector>
#include <memory>
#include <algorithm>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "nav2_util/odometry_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"

#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "cm_msgs/srv/auto_move.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <angles/angles.h>

namespace nav2_backup_controller
{
class BackupController
{
public:

  BackupController(const nav2_util::LifecycleNode::WeakPtr & node,
      const std::string & name,const std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &costmap_ros);

  ~BackupController(){};

  /**
   * @brief Configure controller state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure();



  bool AutoMove(
    const std::shared_ptr<cm_msgs::srv::AutoMove::Request> request,
    std::shared_ptr<cm_msgs::srv::AutoMove::Response> response);

  bool Motion(double motion_x, double motion_y, double motion_theta);

  bool getOdomPose(geometry_msgs::msg::PoseStamped& odom_pose);

  void moveBack(const double back_dist);

  void ComputeRotVel(const double rot_angle);

  void publishVelocity(const geometry_msgs::msg::TwistStamped & velocity);
  /**
   * @brief Calls velocity publisher to publish zero velocity
   */
  void publishZeroVelocity();

  void activate();

  void deactivate();

  void cleanup();

  bool getAntiCollision();

  void BackupThreadLoop();


protected:

  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_; 
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_;
  rclcpp::Logger logger_ {rclcpp::get_logger("BackupController")};
  rclcpp::Clock::SharedPtr clock_;

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>> vel_publisher_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int32>> rotation_status_pub_;
  //rclcpp::Service<cm_msgs::srv::AutoMove>::SharedPtr anti_collision_service_;
  rclcpp::Service<cm_msgs::srv::AutoMove>::SharedPtr anti_collision_service_;

  //std::recursive_mutex backup_mutex_;
  std::mutex backup_mutex_;
  std::condition_variable_any backup_cond_;
  //std::shared_ptr<nav2_backup_controller::BackupController> backup_controller_;
  std::shared_ptr<std::thread> backup_thread_;
  bool backup_thread_shutdown_{false};

  tf2::Duration transform_tolerance_;
  bool anti_collision_;
  bool move_back_success_;
  bool computeRotVel_success_;
  double motion_x_;
  double motion_y_;
  double motion_theta_;
  double back_up_vel_ ; //to be param;
  double back_up_dist_ ; //tobeparam
  double max_vel_theta_ ;//tobeparam
  double rot_tolerance_ ;
  double acc_lim_theta_ ;
  double min_vel_theta_ ;
  double controller_frequency_;
  bool backup_action_;


};

} 

#endif  
