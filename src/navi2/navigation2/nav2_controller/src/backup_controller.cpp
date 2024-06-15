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

#include <algorithm>
#include <string>
#include <limits>
#include <memory>
#include <utility>

#include "nav2_controller/backup_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_costmap_2d/costmap_filters/filter_values.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;
using namespace nav2_costmap_2d;  // NOLINT

namespace nav2_backup_controller
{

BackupController::BackupController(const nav2_util::LifecycleNode::WeakPtr & node,
      const std::string & name,const std::shared_ptr<tf2_ros::Buffer> & tf_buffer,
      const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
      : node_(node), name_(name),tf_(tf_buffer),costmap_ros_(costmap_ros)
{
  RCLCPP_WARN(logger_, "[%s]: 233Creating BackupController", name_.c_str());
  anti_collision_ = false;
  motion_x_ = 0.0;
  motion_y_ = 0.0;
  motion_theta_ = 0.0;
  backup_action_ = false;
}


void BackupController::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }

  costmap_ = costmap_ros_->getCostmap();
  clock_ = node->get_clock();

  vel_publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
  rotation_status_pub_ = node->create_publisher<std_msgs::msg::Int32>("rotation_status",10);
  
  RCLCPP_WARN(logger_,"backup controller cofingrue");
  // anti_collision_service_ = node->create_service<cm_msgs::srv::AutoMove>(
  //   "AutoMove",
  //   std::bind(&BackupController::AutoMove,this,
  //   std::placeholders::_1,std::placeholders::_2));

  nav2_util::declare_parameter_if_not_declared(
  node, name_ + ".back_up_vel", rclcpp::ParameterValue(-0.1));
  back_up_vel_ = node->get_parameter(name_ + ".back_up_vel").as_double();

  nav2_util::declare_parameter_if_not_declared(
  node, name_ + ".acc_lim_theta", rclcpp::ParameterValue(0.6));
  acc_lim_theta_ = node->get_parameter(name_ + ".acc_lim_theta").as_double();

  nav2_util::declare_parameter_if_not_declared(
  node, name_ + ".min_vel_theta", rclcpp::ParameterValue(0.2));
  min_vel_theta_ = node->get_parameter(name_ + ".min_vel_theta").as_double();

  nav2_util::declare_parameter_if_not_declared(
  node, name_ + ".max_vel_theta", rclcpp::ParameterValue(0.4));
  max_vel_theta_ = node->get_parameter(name_ + ".max_vel_theta").as_double();

  nav2_util::declare_parameter_if_not_declared(
  node, name_ + ".rot_tolerance", rclcpp::ParameterValue(0.14));
  rot_tolerance_ = node->get_parameter(name_ + ".rot_tolerance").as_double();

  nav2_util::declare_parameter_if_not_declared(
  node, name_ + ".controller_frequency", rclcpp::ParameterValue(20.0));
  controller_frequency_ = node->get_parameter(name_ + ".controller_frequency").as_double();

  RCLCPP_INFO(logger_,"backup_controller  parameter----back_up_vel:           %f",back_up_vel_);
  RCLCPP_INFO(logger_,"backup_controller  parametee----acc_lim_theta:         %f",acc_lim_theta_);
  RCLCPP_INFO(logger_,"backup_controller  parametee----min_vel_theta:         %f",min_vel_theta_);
  RCLCPP_INFO(logger_,"backup_controller  parametee----max_vel_theta:         %f",max_vel_theta_);
  RCLCPP_INFO(logger_,"backup_controller  parameter----rot_tolerance:         %f",rot_tolerance_);
  RCLCPP_INFO(logger_,"backup_controller  parameter----controler_fre:         %f",controller_frequency_);

}

void BackupController::activate()
{
  RCLCPP_WARN(logger_,"BackupController activate!");
  auto node = node_.lock();
  if (!node) {
    throw nav2_core::PlannerException("Unable to lock node!");
  }
  vel_publisher_->on_activate();
  rotation_status_pub_->on_activate();
  anti_collision_service_ = node->create_service<cm_msgs::srv::AutoMove>(
    "AutoMove",
    std::bind(&BackupController::AutoMove,this,
    std::placeholders::_1,std::placeholders::_2));

  backup_thread_shutdown_ = false;
  backup_thread_ = std::make_shared<std::thread>(std::bind(&BackupController::BackupThreadLoop, this));

}

void BackupController::deactivate()
{
    RCLCPP_WARN(logger_,"BackupController deactivate!");
    vel_publisher_->on_deactivate();
    rotation_status_pub_->on_deactivate();
}

void BackupController::cleanup()
{
  RCLCPP_WARN(logger_,"BackupController cleanup!");
  vel_publisher_.reset();
  rotation_status_pub_.reset();
  if (backup_thread_ && backup_thread_->joinable()) {
    backup_thread_shutdown_ = true;
    {
      std::unique_lock<std::mutex> lock(backup_mutex_);
      anti_collision_ = true;
    }
    backup_cond_.notify_one();

    backup_thread_->join();
    backup_thread_.reset();
  }
  //RCLCPP_WARN(logger_,"BackupController cleanup finish!");
}


void BackupController::BackupThreadLoop()
{
  //std::unique_lock<std::mutex> lock(backup_mutex_);
  while (rclcpp::ok()&&!backup_thread_shutdown_){
    //RCLCPP_WARN(logger_,"BackupController 1111111");
    #if 0
    // if(!anti_collision_){
    //   RCLCPP_INFO(logger_,"suspending BackupThreadLoop!");
    //   backup_cond_.wait(lock);
    // }
    // lock.unlock();
    // Motion(motion_x_,motion_y_,motion_theta_);
    // anti_collision_ = false;
    // lock.lock();
    #else
    {
      std::unique_lock<std::mutex> lock(backup_mutex_);
      backup_cond_.wait(lock, [this]{return anti_collision_;}); //
    }

    Motion(motion_x_,motion_y_,motion_theta_);
    
    motion_x_ = 0.0;
    motion_y_ = 0.0;
    motion_theta_ = 0.0;

    {
      std::unique_lock<std::mutex> lock(backup_mutex_);
      anti_collision_ = false;
      backup_action_ = false;
    }
    #endif

    //backup_action_ = false;
  }
}

bool BackupController::AutoMove(const std::shared_ptr<cm_msgs::srv::AutoMove::Request> request,
   std::shared_ptr<cm_msgs::srv::AutoMove::Response> response)
{
  //backup_action_ = true;
  RCLCPP_WARN_THROTTLE(logger_,*(clock_), 1000, "aauto move servicer start :request %f,%f,%f",
     request->pose2d.x,request->pose2d.y,request->pose2d.theta);
  motion_x_ = request->pose2d.x;
  motion_y_ = request->pose2d.y;
  motion_theta_ = request->pose2d.theta;
  std::unique_lock<std::mutex> lock(backup_mutex_);
  anti_collision_ = true;
  backup_action_ = true;
  backup_cond_.notify_one();
  //lock.unlock();
  response->success = true;
  return true;
}

bool BackupController::getAntiCollision(){
  //return anti_collision_;
  std::unique_lock<std::mutex> lock(backup_mutex_);
  //RCLCPP_WARN(logger_,"BackupController lock");
  return backup_action_; //backup_action_
}

bool BackupController::Motion(double motion_x, double motion_y, double motion_theta)
{
  motion_y = motion_y;
  move_back_success_ = true;
  computeRotVel_success_ = true;
	if(motion_x != 0){
    move_back_success_ = false;
		moveBack(motion_x);
	}
	if(motion_theta != 0){
    computeRotVel_success_ = false;
		ComputeRotVel(motion_theta);
	}
  if(computeRotVel_success_ && move_back_success_ ){
    RCLCPP_WARN(logger_,"backup and rotation motion success");  
  }else{
    RCLCPP_WARN(logger_,"backup and rotation motion failed , may be overtime");  
  }
  //anti_collision_ = false;
  
  return true;
}

void BackupController::ComputeRotVel(const double rot_angle)
{
  RCLCPP_WARN(logger_,"rot_angle %f",rot_angle); 
  std_msgs::msg::Int32 status_msgs;
  status_msgs.data = 1;

  // auto node = shared_from_this();
  // std::shared_ptr<tf2_ros::Buffer> tfBuffer = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  // tfBuffer->setUsingDedicatedThread(true);
  // std::shared_ptr<tf2_ros::TransformListener> tf_listener;
  // tf_listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);
  
  
  //std::shared_ptr<tf2_ros::Buffer> tfBuffer = costmap_ros_->getTfBuffer();
  geometry_msgs::msg::TransformStamped current_pose;
  try{
    current_pose =  tf_->lookupTransform("odom","base_link",tf2::TimePointZero,tf2::durationFromSec(0.5));
  }catch(tf2::TransformException & ex){
    RCLCPP_ERROR(logger_,"computere rot vel: %s",ex.what());
    status_msgs.data = -1;
    rotation_status_pub_->publish(status_msgs);
    return;
  }


  double start_angle = tf2::getYaw(current_pose.transform.rotation);
  double target_angle = start_angle + rot_angle;
  target_angle = angles::normalize_angle(target_angle);

  //bool reach_target = false;
  rclcpp::Time start_time = clock_->now();
  double need_time = 2.0 * M_PI / max_vel_theta_;
  rclcpp::Time end_time = start_time + rclcpp::Duration::from_seconds(need_time);
  rotation_status_pub_->publish(status_msgs);
  rclcpp::WallRate loop_rate(controller_frequency_);
  while(rclcpp::ok()){
    if(clock_->now() > end_time){
      RCLCPP_ERROR(logger_,"rotate , time out");
      geometry_msgs::msg::TwistStamped cmd_vel;
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.linear.z = 0.0;
      cmd_vel.twist.angular.x = 0.0;
      cmd_vel.twist.angular.y = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel.header.stamp = clock_->now();
      publishVelocity(cmd_vel);     
      status_msgs.data = -1;
      break;
    }

    try{
      current_pose =  tf_->lookupTransform("odom","base_link",tf2::TimePointZero,tf2::durationFromSec(0.5));
    }catch(tf2::TransformException & ex){
      RCLCPP_ERROR(logger_,"computere rot vel: %s",ex.what());
      continue;
    }

		double current_angle = tf2::getYaw(current_pose.transform.rotation);
		double angle_to_target = angles::shortest_angular_distance(current_angle, target_angle);
		if (std::fabs(angle_to_target) < rot_tolerance_) {
			RCLCPP_INFO_THROTTLE(logger_,*(clock_), 1000,"reach target angle: %f", angle_to_target);
      geometry_msgs::msg::TwistStamped cmd_vel;
      cmd_vel.twist.linear.x = 0.0;
      cmd_vel.twist.linear.y = 0.0;
      cmd_vel.twist.linear.z = 0.0;
      cmd_vel.twist.angular.x = 0.0;
      cmd_vel.twist.angular.y = 0.0;
      cmd_vel.twist.angular.z = 0.0;
      cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
      cmd_vel.header.stamp = clock_->now();
      publishVelocity(cmd_vel);     
			status_msgs.data = 2;
      computeRotVel_success_ = true;
			break;
		}
		double angle_to_start = angles::shortest_angular_distance(start_angle, current_angle);
		double vel_1 = std::sqrt(2 * acc_lim_theta_ * std::fabs(angle_to_start));
		double vel_2 = std::sqrt(2 * acc_lim_theta_ * std::fabs(angle_to_target));
		double vel = std::min(vel_1, vel_2);
		// make sure that this velocity falls within the specified limits
		vel = std::min(std::max(vel, min_vel_theta_), max_vel_theta_);
		vel = rot_angle > 0 ? vel : -vel; // direct

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.twist.linear.x = 0.0;
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.linear.z = 0.0;
    cmd_vel.twist.angular.x = 0.0;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = vel;
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel.header.stamp = clock_->now();
    publishVelocity(cmd_vel); 
    //action_server_->succeeded_current();

    if (!loop_rate.sleep()) {
      RCLCPP_WARN(
        logger_, "backup Control loop missed its desired rate of %.4fHz",
        controller_frequency_);
    }
  }
  rotation_status_pub_->publish(status_msgs);
  return;
}

void BackupController::moveBack(const double back_dist)
{
  RCLCPP_WARN(logger_,"back_dist %f",back_dist);  
  rclcpp::Time back_up_start_time = clock_->now();
  std_msgs::msg::Int32 status_msgs;
  status_msgs.data = 1;
  geometry_msgs::msg::PoseStamped start_pose;
  if(!getOdomPose(start_pose)){
    RCLCPP_ERROR(logger_,"Can not get odom pose!");  
  }
  geometry_msgs::msg::PoseStamped robot_current_vel;
  //double total_dist = 0;
  rclcpp::WallRate loop_rate(controller_frequency_);
  while(rclcpp::ok()){
    geometry_msgs::msg::TwistStamped cmd_vel;
    if(back_dist < 0){
      cmd_vel.twist.linear.x = back_up_vel_;
    }
    else{
      cmd_vel.twist.linear.x = -(back_up_vel_);
    }
    cmd_vel.twist.linear.y = 0.0;
    cmd_vel.twist.linear.z = 0.0;
    cmd_vel.twist.angular.x = 0.0;
    cmd_vel.twist.angular.y = 0.0;
    cmd_vel.twist.angular.z = 0.0;
    cmd_vel.header.frame_id = costmap_ros_->getBaseFrameID();
    cmd_vel.header.stamp = clock_->now();
    publishVelocity(cmd_vel);


    geometry_msgs::msg::PoseStamped cur_pose;
    if(!getOdomPose(cur_pose)){
      RCLCPP_ERROR(logger_,"Can not get odom pose!");  
    }
    double move_back_dist = sqrt(pow(cur_pose.pose.position.x - start_pose.pose.position.x,2)
                                + pow(cur_pose.pose.position.y - start_pose.pose.position.y,2));
    RCLCPP_INFO_THROTTLE(logger_,*(clock_), 1000,"move back dist : %f, back dist: %f",move_back_dist,back_dist);
    if(move_back_dist > fabs(back_dist)){
      RCLCPP_INFO(logger_,"back up action has reached %f",back_dist);
      // status_msgs.data = 2;
      // rotation_status_pub_.publish(status_msgs);
      move_back_success_ = true;
      publishZeroVelocity();
      break;
    }
    double run_time = (clock_->now() - back_up_start_time).seconds();
    if(run_time > 10.0){//tobeparam
      status_msgs.data = -1;
      //tobe add status publisth
      rotation_status_pub_->publish(status_msgs);
      RCLCPP_ERROR(logger_,"back up out of time ,back up fail!!!");
      break;
    }
    //action_server_->succeeded_current();
    if (!loop_rate.sleep()) {
      RCLCPP_WARN(
        logger_, "backup Control loop missed its desired rate of %.4fHz",
        controller_frequency_);
    }
  }
  return;
}

bool BackupController::getOdomPose(geometry_msgs::msg::PoseStamped& odom_pose)
{

  //RCLCPP_INFO_ONCE(logger_,"doom: %f",odom_pose.pose.position.x);

  geometry_msgs::msg::TransformStamped transform;
  try{
    transform =  tf_->lookupTransform("odom","base_link",tf2::TimePointZero,tf2::durationFromSec(0.5));
  }catch(tf2::TransformException & ex){
    RCLCPP_ERROR(logger_,"computere rot vel: %s",ex.what());
    return false;
  }
  geometry_msgs::msg::Quaternion q;
  q.w = transform.transform.rotation.w;
  q.x = transform.transform.rotation.x;
  q.y = transform.transform.rotation.y;
  q.z = transform.transform.rotation.z;

  odom_pose.pose.position.x = transform.transform.translation.x;
  odom_pose.pose.position.y = transform.transform.translation.y;
  odom_pose.pose.position.z = transform.transform.translation.z;
  odom_pose.pose.orientation.x = transform.transform.rotation.x;
  odom_pose.pose.orientation.y = transform.transform.rotation.y;
  odom_pose.pose.orientation.z = transform.transform.rotation.z;
  odom_pose.pose.orientation.w = transform.transform.rotation.w;

  return true;
}



void BackupController::publishVelocity(const geometry_msgs::msg::TwistStamped & velocity)
{
  auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(velocity.twist);
  if (vel_publisher_->is_activated() && vel_publisher_->get_subscription_count() > 0) {
      vel_publisher_->publish(std::move(cmd_vel));
  }
  RCLCPP_INFO_THROTTLE(logger_,*(clock_), 1000,"pub velocity: %f,%f,%f",velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.angular.z);
}

void BackupController::publishZeroVelocity()
{
  geometry_msgs::msg::TwistStamped velocity;
  velocity.twist.angular.x = 0;
  velocity.twist.angular.y = 0;
  velocity.twist.angular.z = 0;
  velocity.twist.linear.x = 0;
  velocity.twist.linear.y = 0;
  velocity.twist.linear.z = 0;
  velocity.header.frame_id = costmap_ros_->getBaseFrameID();
  velocity.header.stamp = clock_->now();
  RCLCPP_INFO_THROTTLE(logger_,*(clock_), 1000,"pubzero velocity: %f,%f,%f",velocity.twist.linear.x,velocity.twist.linear.y,velocity.twist.angular.z);
  publishVelocity(velocity);
}
}

