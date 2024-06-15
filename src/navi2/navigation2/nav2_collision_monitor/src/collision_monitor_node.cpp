// Copyright (c) 2022 Samsung Research Russia
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

#include "nav2_collision_monitor/collision_monitor_node.hpp"

#include <exception>
#include <utility>
#include <functional>

#include "tf2_ros/create_timer_ros.h"

#include "nav2_util/node_utils.hpp"

#include "nav2_collision_monitor/kinematics.hpp"
using namespace std::chrono_literals;

namespace nav2_collision_monitor
{

CollisionMonitor::CollisionMonitor(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("collision_monitor", "", true, options),
  process_active_(false), robot_action_prev_{DO_NOTHING, {-1.0, -1.0, -1.0}},
  stop_stamp_{0, 0, get_clock()->get_clock_type()}, stop_pub_timeout_(1.0, 0.0)
{
  clock_ = this->get_clock();
}

CollisionMonitor::~CollisionMonitor()
{
  polygons_.clear();
  sources_.clear();
  inflexible_detector_.reset();
  if(visualdrop_cmd_lock_timer_){
    visualdrop_cmd_lock_timer_->cancel();
    visualdrop_cmd_lock_timer_.reset();
  }
  if(ultra_cmd_lock_timer_){
    ultra_cmd_lock_timer_->cancel();
    ultra_cmd_lock_timer_.reset();
  }
}

nav2_util::CallbackReturn
CollisionMonitor::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Transform buffer and listener initialization
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  first_visualdrop_cmd_lock_ = true;
  first_ultra_cmd_lock_ = true;
  last_check_safety_ = true;
  //last_check_drop_detection_ = true;
  drop_detection_ = false;

  // set emergerncy stop cliet
  // emergency_stop_client_ = this->create_client<cm_msgs::srv::set_string_stamped>("/emergency_stop");
  // last_call_emergency_time_ = rclcpp::Time();

  std::string cmd_vel_in_topic;
  std::string cmd_vel_out_topic;
  std::string cmd_vel_out_topic_twiststamped;
  

  // Obtaining ROS parameters
  if (!getParameters(cmd_vel_in_topic, cmd_vel_out_topic,cmd_vel_out_topic_twiststamped)) {
    return nav2_util::CallbackReturn::FAILURE;
  }

  cmd_vel_in_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    cmd_vel_in_topic, 1,
    std::bind(&CollisionMonitor::cmdVelInCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1,
    std::bind(&CollisionMonitor::odomCallback, this, std::placeholders::_1));

  // std::string ultras_topic = "all_ultra_data";
  // ultras_sub_ = this->create_subscription<sensor_msgs::msg::ChannelFloat32>(
  //   "all_ultra_data",1,
  //   std::bind(&CollisionMonitor::RangesMsgCallback,this,std::placeholders::_1));


  cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
    cmd_vel_out_topic, 1);
  cmd_vel_out_twiststamped_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    cmd_vel_out_topic_twiststamped,1);
  cmd_lock_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
    cmd_lock_topic_,1);

  //add drop_detection_sub_
  drop_detection_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/VisualDrop", 1,
    std::bind(&CollisionMonitor::VisualDropCallback, this, std::placeholders::_1));

  battery_state_sub_ = this->create_subscription<cm_msgs::msg::BatteryState>(
    "/BatteryState", 1,
    std::bind(&CollisionMonitor::BatteryStateCallback, this, std::placeholders::_1));

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Activating lifecycle publisher
  cmd_vel_out_pub_->on_activate();
  cmd_vel_out_twiststamped_pub_->on_activate();
  cmd_lock_pub_->on_activate();

  // Activating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->activate();
  }

  inflexible_detector_->activate();

  // Since polygons are being published when cmd_vel_in appears,
  // we need to publish polygons first time to display them at startup
  publishPolygons();

  // Activating main worker
  process_active_ = true;

  // Creating bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  // Deactivating main worker
  process_active_ = false;

  // Reset action type to default after worker deactivating
  robot_action_prev_ = {DO_NOTHING, {-1.0, -1.0, -1.0}};

  // Deactivating polygons
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->deactivate();
  }

  // Deactivating lifecycle publishers
  cmd_vel_out_pub_->on_deactivate();
  cmd_vel_out_twiststamped_pub_->on_deactivate();
  cmd_lock_pub_->on_deactivate();

  inflexible_detector_->deactivate();

  // Destroying bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  cmd_vel_in_sub_.reset();
  odom_sub_.reset();
  drop_detection_sub_.reset();
  battery_state_sub_.reset();
  cmd_vel_out_pub_.reset();
  cmd_vel_out_twiststamped_pub_.reset();
  cmd_lock_pub_.reset();

  polygons_.clear();
  sources_.clear();

  inflexible_detector_.reset();

  tf_listener_.reset();
  tf_buffer_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
CollisionMonitor::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");

  return nav2_util::CallbackReturn::SUCCESS;
}

void CollisionMonitor::BatteryStateCallback(cm_msgs::msg::BatteryState::ConstSharedPtr msg)
{
  if(last_charge_type_ == -1 && msg->chargeset_type != -1){
    //pub unlock
    RCLCPP_WARN(get_logger(),"ROBOT IS CHARGING! UNLOCK, cmd_lock pub linear.z = 0!!!");
    last_charge_type_ = msg->chargeset_type;
    Unlockcmd();
  }else{
    last_charge_type_ = msg->chargeset_type;
  }
}

void CollisionMonitor::Unlockcmd()
{
  //unlock ,pub z=0.0;
  std::unique_ptr<geometry_msgs::msg::TwistStamped> cmd_lock_pub_msg =
    std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_lock_pub_msg->twist.linear.z = 0.0;
  cmd_lock_pub_msg->header.frame_id = "navigation";
  cmd_lock_pub_msg->header.stamp = this->now();

  cmd_lock_pub_->publish(std::move(cmd_lock_pub_msg));    
}

void CollisionMonitor::VisualDropCallback(std_msgs::msg::Int32::ConstSharedPtr msg)
{
  if(!drop_detection_switch_){
    return;
  }

  if(msg->data > 0)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),*(clock_),2000,"VisualDrop detection ,pub cmd_lock");
    //last_check_drop_detection_ = false;
    drop_detection_ = true;
    if(first_visualdrop_cmd_lock_){
      visualdrop_cmd_lock_timer_ = this->create_wall_timer(100ms,std::bind(&CollisionMonitor::VisualdropCmdlockTimerCallback, this));
      first_visualdrop_cmd_lock_ = false;
    }
  }
  else if(msg->data == 0)
  {
    first_visualdrop_cmd_lock_ = true;
    //drop_detection_ = false;
    //last state is unsafety, now safety,safety state change, cmd_lock pub linear.z = 0;
    if(drop_detection_){  
      RCLCPP_WARN(get_logger(),"VisualDrop unlock, cmd_lock pub linear.z = 0!!!");
      //reset visualdrop_cmd_lock_timer_
      if(visualdrop_cmd_lock_timer_){
        visualdrop_cmd_lock_timer_->cancel();
        visualdrop_cmd_lock_timer_.reset();
      }
      //last_check_drop_detection_ = true;
      drop_detection_ = false;
      //cmd_lock_pub_
      std::unique_ptr<geometry_msgs::msg::TwistStamped> cmd_lock_pub_msg =
        std::make_unique<geometry_msgs::msg::TwistStamped>();
      cmd_lock_pub_msg->twist.linear.z = 0.0;
      cmd_lock_pub_msg->header.frame_id = "navigation";
      cmd_lock_pub_msg->header.stamp = this->now();

      cmd_lock_pub_->publish(std::move(cmd_lock_pub_msg));   
    }
  }
}

void CollisionMonitor::odomCallback(nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  odom_vel_.x = msg->twist.twist.linear.x;
  odom_vel_.y = msg->twist.twist.linear.y;
  odom_vel_.tw = msg->twist.twist.angular.z;
}

void CollisionMonitor::cmdVelInCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  process({msg->linear.x, msg->linear.y, msg->angular.z});
}

// void CollisionMonitor::RangesMsgCallback(sensor_msgs::msg::ChannelFloat32::ConstSharedPtr ranges_message)
// {
//   sensor_msgs::msg::ChannelFloat32 ultra_datas =  *ranges_message;
//   inflexible_detector_->SetUltradatas(ultra_datas);
// }

void CollisionMonitor::publishVelocity(const Action & robot_action)
{
  //RCLCPP_INFO_THROTTLE(get_logger(),*(clock_),2000,"publish normal vel,topic: /chassis/cmd");
  if (robot_action.req_vel.isZero()) {
    if (!robot_action_prev_.req_vel.isZero()) {
      // Robot just stopped: saving stop timestamp and continue
      stop_stamp_ = this->now();
    } else if (this->now() - stop_stamp_ > stop_pub_timeout_) {
      // More than stop_pub_timeout_ passed after robot has been stopped.
      // Cease publishing output cmd_vel.
      return;
    }
  }

  std::unique_ptr<geometry_msgs::msg::Twist> cmd_vel_out_msg =
    std::make_unique<geometry_msgs::msg::Twist>();
  cmd_vel_out_msg->linear.x = robot_action.req_vel.x;
  cmd_vel_out_msg->linear.y = robot_action.req_vel.y;
  cmd_vel_out_msg->angular.z = robot_action.req_vel.tw;
  // linear.z, angular.x and angular.y will remain 0.0

  std::unique_ptr<geometry_msgs::msg::TwistStamped> cmd_vel_out_twiststamped_msg =
    std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_vel_out_twiststamped_msg->twist.linear.x = robot_action.req_vel.x;
  cmd_vel_out_twiststamped_msg->twist.linear.y = robot_action.req_vel.y;
  cmd_vel_out_twiststamped_msg->twist.angular.z = robot_action.req_vel.tw;
  cmd_vel_out_twiststamped_msg->header.frame_id = "navigation";
  cmd_vel_out_twiststamped_msg->header.stamp = this->now();

  cmd_vel_out_pub_->publish(std::move(cmd_vel_out_msg));
  cmd_vel_out_twiststamped_pub_->publish(std::move(cmd_vel_out_twiststamped_msg));
}

bool CollisionMonitor::getParameters(
  std::string & cmd_vel_in_topic,
  std::string & cmd_vel_out_topic,
  std::string & cmd_vel_out_topic_twiststamped)
{
  std::string base_frame_id, odom_frame_id;
  tf2::Duration transform_tolerance;
  rclcpp::Duration source_timeout(2.0, 0.0);

  auto node = shared_from_this();



  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_lock_topic", rclcpp::ParameterValue("chassis/cmd_lock"));
  cmd_lock_topic_ = get_parameter("cmd_lock_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_in_topic", rclcpp::ParameterValue("cmd_vel_raw"));
  cmd_vel_in_topic = get_parameter("cmd_vel_in_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_out_topic", rclcpp::ParameterValue("cmd_vel"));
  cmd_vel_out_topic = get_parameter("cmd_vel_out_topic").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "cmd_vel_out_topic_twiststamped", rclcpp::ParameterValue("cmd_vel_stamp"));
  cmd_vel_out_topic_twiststamped = get_parameter("cmd_vel_out_topic_twiststamped").as_string();

  nav2_util::declare_parameter_if_not_declared(
    node, "base_frame_id", rclcpp::ParameterValue("base_footprint"));
  base_frame_id = get_parameter("base_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "odom_frame_id", rclcpp::ParameterValue("odom"));
  odom_frame_id = get_parameter("odom_frame_id").as_string();
  nav2_util::declare_parameter_if_not_declared(
    node, "transform_tolerance", rclcpp::ParameterValue(0.1));
  transform_tolerance =
    tf2::durationFromSec(get_parameter("transform_tolerance").as_double());
  nav2_util::declare_parameter_if_not_declared(
    node, "source_timeout", rclcpp::ParameterValue(2.0));
  source_timeout =
    rclcpp::Duration::from_seconds(get_parameter("source_timeout").as_double());

  nav2_util::declare_parameter_if_not_declared(
    node, "stop_pub_timeout", rclcpp::ParameterValue(1.0));
  stop_pub_timeout_ =
    rclcpp::Duration::from_seconds(get_parameter("stop_pub_timeout").as_double());

  nav2_util::declare_parameter_if_not_declared(
    node, "max_vel", rclcpp::ParameterValue(0.7));
  max_vel_ =get_parameter("max_vel").as_double();  

  nav2_util::declare_parameter_if_not_declared(
  node, "drop_detection_switch", rclcpp::ParameterValue(true));
  drop_detection_switch_ = node->get_parameter("drop_detection_switch").as_bool();

  nav2_util::declare_parameter_if_not_declared(
    node, "car_type", rclcpp::ParameterValue("ginger_lite"));
  car_type_ = get_parameter("car_type").as_string();

  RCLCPP_INFO(get_logger(),"collision_monitor common parameter----cmd_lock_topic:                   %s",cmd_lock_topic_.c_str());
  RCLCPP_INFO(get_logger(),"collision_monitor common parameter----drop_detection_switch:            %d",drop_detection_switch_);
  RCLCPP_INFO(get_logger(),"collision_monitor common parameter----car_type:                         %s",car_type_.c_str());

  if(!configureInflexibleDetection()){
    return false;
  }

  if (!configurePolygons(base_frame_id, transform_tolerance)) {
    return false;
  }

  if (!configureSources(base_frame_id, odom_frame_id, transform_tolerance, source_timeout)) {
    return false;
  }

  return true;
}

bool CollisionMonitor::configureInflexibleDetection(){
  try{
    auto node = shared_from_this();
    std::string name = "inflexible_detection";
    inflexible_detector_ = std::make_shared<InflexibleDetection>(node,name,tf_buffer_);
    if(!inflexible_detector_->configure()){
      return false;
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }
  return true;
}

bool CollisionMonitor::configurePolygons(
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
{
  try {
    auto node = shared_from_this();

    nav2_util::declare_parameter_if_not_declared(
      node, "polygons", rclcpp::ParameterValue(std::vector<std::string>()));
    std::vector<std::string> polygon_names = get_parameter("polygons").as_string_array();
    for (std::string polygon_name : polygon_names) {
      // Leave it not initialized: the will cause an error if it will not set
      nav2_util::declare_parameter_if_not_declared(
        node, polygon_name + ".type", rclcpp::PARAMETER_STRING);
      const std::string polygon_type = get_parameter(polygon_name + ".type").as_string();

      if (polygon_type == "polygon") {
        polygons_.push_back(
          std::make_shared<Polygon>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else if (polygon_type == "circle") {
        polygons_.push_back(
          std::make_shared<Circle>(
            node, polygon_name, tf_buffer_, base_frame_id, transform_tolerance));
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown polygon type: %s",
          polygon_name.c_str(), polygon_type.c_str());
        return false;
      }

      // Configure last added polygon
      if (!polygons_.back()->configure()) {
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

bool CollisionMonitor::configureSources(
  const std::string & base_frame_id,
  const std::string & odom_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout)
{
  try {
    auto node = shared_from_this();

    // Leave it to be not initialized: to intentionally cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, "observation_sources", rclcpp::PARAMETER_STRING_ARRAY);
    std::vector<std::string> source_names = get_parameter("observation_sources").as_string_array();
    for (std::string source_name : source_names) {
      nav2_util::declare_parameter_if_not_declared(
        node, source_name + ".type",
        rclcpp::ParameterValue("scan"));  // Laser scanner by default
      const std::string source_type = get_parameter(source_name + ".type").as_string();

      if (source_type == "scan") {
        std::shared_ptr<Scan> s = std::make_shared<Scan>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout);

        s->configure();

        sources_.push_back(s);
      } else if (source_type == "pointcloud") {
        std::shared_ptr<PointCloud> p = std::make_shared<PointCloud>(
          node, source_name, tf_buffer_, base_frame_id, odom_frame_id,
          transform_tolerance, source_timeout);

        p->configure();

        sources_.push_back(p);
      } else {  // Error if something else
        RCLCPP_ERROR(
          get_logger(),
          "[%s]: Unknown source type: %s",
          source_name.c_str(), source_type.c_str());
        return false;
      }
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while getting parameters: %s", ex.what());
    return false;
  }

  return true;
}

void CollisionMonitor::VisualdropCmdlockTimerCallback()
{
  //RCLCPP_WARN_THROTTLE(get_logger(),*(clock_),2000,"2s :emergency stop, 10hz pub cmd_lock!!!");
  //RCLCPP_WARN(get_logger(),"emergency stop, 10hz pub cmd_lock!!!");
  //cmd_lock_pub_
  std::unique_ptr<geometry_msgs::msg::TwistStamped> cmd_lock_pub_msg =
    std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_lock_pub_msg->twist.linear.z = 1.0;
  cmd_lock_pub_msg->header.frame_id = "navigation";
  //cmd_lock_pub_msg->header.stamp = rclcpp::Time();
  cmd_lock_pub_msg->header.stamp = this->now();

  cmd_lock_pub_->publish(std::move(cmd_lock_pub_msg));   
}

void CollisionMonitor::UltraCmdlockTimerCallback()
{
  std::unique_ptr<geometry_msgs::msg::TwistStamped> cmd_lock_pub_msg =
    std::make_unique<geometry_msgs::msg::TwistStamped>();
  cmd_lock_pub_msg->twist.linear.z = 1.0;
  cmd_lock_pub_msg->header.frame_id = "navigation";
  cmd_lock_pub_msg->header.stamp = this->now();

  cmd_lock_pub_->publish(std::move(cmd_lock_pub_msg));    
}

void CollisionMonitor::process(const Velocity & cmd_vel_in)
{
  // Current timestamp for all inner routines prolongation
  rclcpp::Time curr_time = this->now();

  // Do nothing if main worker in non-active state
  if (!process_active_) {
    return;
  }

  if(drop_detection_){
    return;
  }
  
  //RCLCPP_WARN(get_logger(),"emergency stop, 10hz pub cmd_lock!!!");
  //check ultras_data collision
  geometry_msgs::msg::Twist cmd_vel_inflexible_detection;
  cmd_vel_inflexible_detection.linear.x = cmd_vel_in.x;
  cmd_vel_inflexible_detection.linear.y = cmd_vel_in.y;
  cmd_vel_inflexible_detection.angular.z = cmd_vel_in.tw;

  #if 1
  if(!inflexible_detector_->CheckRangeSafety(cmd_vel_inflexible_detection,car_type_)){
    RCLCPP_WARN(get_logger(),"inflexible_detector_ CheckRangeSafety(ultra) is unsafety ,pub cmd_lock!");
    last_check_safety_ = false;
    ultra_unlock_count_ = 0;
    if(first_ultra_cmd_lock_){
      ultra_cmd_lock_timer_ = this->create_wall_timer(100ms,std::bind(&CollisionMonitor::UltraCmdlockTimerCallback, this));
      first_ultra_cmd_lock_ = false;

    }
    return; 
  }else{
    
    //last state is unsafety, now safety,safety state change, cmd_lock pub linear.z = 0,unlock
    if(!last_check_safety_){
      ultra_unlock_count_++;
      RCLCPP_WARN(get_logger(),"safety state change to safety(ultra), unlock_count: %d",ultra_unlock_count_);
      if(ultra_unlock_count_ == 10){
        RCLCPP_WARN(get_logger(),"safety state change to safety(ultra), cmd_lock pub linear.z = 0, unlock!!!");
        //reset first_ultra_cmd_lock_
        if(ultra_cmd_lock_timer_){
          ultra_cmd_lock_timer_->cancel();
          ultra_cmd_lock_timer_.reset();
        }
        first_ultra_cmd_lock_ = true;
        last_check_safety_ = true;
        //cmd_lock_pub_  unlock
        std::unique_ptr<geometry_msgs::msg::TwistStamped> cmd_lock_pub_msg =
          std::make_unique<geometry_msgs::msg::TwistStamped>();
        cmd_lock_pub_msg->twist.linear.z = 0.0;
        cmd_lock_pub_msg->header.frame_id = "navigation";
        cmd_lock_pub_msg->header.stamp = this->now();

        cmd_lock_pub_->publish(std::move(cmd_lock_pub_msg)); 
      }  
  
    }
  }
  #else
  if(!inflexible_detector_->CheckRangeSafety(cmd_vel_inflexible_detection,car_type_)){
    RCLCPP_WARN(get_logger(),"inflexible_detector_ CheckRangeSafety is unsafety ,pubish emergency stop!");
    //pub chassis_cmd_vel
    // std::unique_ptr<geometry_msgs::msg::TwistStamped> cmd_vel_out_twiststamped_msg =
    //   std::make_unique<geometry_msgs::msg::TwistStamped>();
    // cmd_vel_out_twiststamped_msg->twist.angular.x = 2.0;
    // cmd_vel_out_twiststamped_msg->twist.angular.y = 2.0;
    // cmd_vel_out_twiststamped_msg->twist.angular.z = 0.0;
    // cmd_vel_out_twiststamped_msg->header.frame_id = "navigation";
    // cmd_vel_out_twiststamped_msg->header.stamp = rclcpp::Time();
    // cmd_vel_out_twiststamped_pub_->publish(std::move(cmd_vel_out_twiststamped_msg));   
    
    //pub cmd_lock


    return; 
  }else{
    //reset ultra_cmd_lock

  }
  #endif

  // Points array collected from different data sources in a robot base frame
  std::vector<Point> collision_points;

  // Fill collision_points array from different data sources
  for (std::shared_ptr<Source> source : sources_) {
    source->getData(curr_time, collision_points);
  }

  // By default - there is no action
  Action robot_action{DO_NOTHING, cmd_vel_in};
  // Polygon causing robot action (if any)
  std::shared_ptr<Polygon> action_polygon;

  for (std::shared_ptr<Polygon> polygon : polygons_) {
    if (robot_action.action_type == STOP) {
      // If robot already should stop, do nothing
      break;
    }

    const ActionType at = polygon->getActionType();
    if (at == STOP || at == SLOWDOWN) {
      // Process STOP/SLOWDOWN for the selected polygon
      if (processDynamicStopSlowdown(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    } else if (at == APPROACH) {
      // Process APPROACH for the selected polygon
      if (processApproach(polygon, collision_points, cmd_vel_in, robot_action)) {
        action_polygon = polygon;
      }
    }
  }

  if (robot_action.action_type != robot_action_prev_.action_type) {
    // Report changed robot behavior
    printAction(robot_action, action_polygon);
  }

  // Publish requred robot velocity
  publishVelocity(robot_action);

  // Publish polygons for better visualization
  publishPolygons();

  robot_action_prev_ = robot_action;
}

bool CollisionMonitor::processStopSlowdown(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  if (polygon->getPointsInside(collision_points) > polygon->getMaxPoints()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else {  // SLOWDOWN
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processDynamicStopSlowdown(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  RCLCPP_INFO(rclcpp::get_logger("collisoin_monitor"),"aciton type = %d",polygon->getActionType());
  if(polygon->getActionType() == STOP){
    RCLCPP_INFO(rclcpp::get_logger("collisoin_monitor"),"goto stop");
    polygon->updatePolygonStop(velocity,max_vel_);
  }else if(polygon->getActionType() == SLOWDOWN){
    RCLCPP_INFO(rclcpp::get_logger("collisoin_monitor"),"goto slowdown");
    polygon->updatePolygonSlowdown(velocity,max_vel_);
  }
  if (polygon->getPointsInside(collision_points) > polygon->getMaxPoints()) {
    if (polygon->getActionType() == STOP) {
      // Setting up zero velocity for STOP model
      RCLCPP_INFO(rclcpp::get_logger("collisoin_monitor"),"goto stop!!!");
      robot_action.action_type = STOP;
      robot_action.req_vel.x = 0.0;
      robot_action.req_vel.y = 0.0;
      robot_action.req_vel.tw = 0.0;
      return true;
    } else {  // SLOWDOWN
      const Velocity safe_vel = velocity * polygon->getSlowdownRatio();
      // Check that currently calculated velocity is safer than
      // chosen for previous shapes one
      if (safe_vel < robot_action.req_vel) {
        robot_action.action_type = SLOWDOWN;
        robot_action.req_vel = safe_vel;
        return true;
      }
    }
  }

  return false;
}

bool CollisionMonitor::processApproach(
  const std::shared_ptr<Polygon> polygon,
  const std::vector<Point> & collision_points,
  const Velocity & velocity,
  Action & robot_action) const
{
  polygon->updatePolygon();

  // Obtain time before a collision
  const double collision_time_odom = polygon->getCollisionTime(collision_points, odom_vel_);
  const double collision_time_vel = polygon->getCollisionTime(collision_points, velocity);
  const double collision_time = (collision_time_vel > collision_time_odom)? collision_time_odom : collision_time_vel;
  if(collision_time <= 0.8){
    RCLCPP_INFO_THROTTLE(get_logger(),*(clock_),2000,"robot get collision after %f seconds!",collision_time);
  }else if(collision_time <=1.6){
    RCLCPP_INFO_THROTTLE(get_logger(),*(clock_),5000,"robot get collison after %f seconds!",collision_time);
  }else{
    RCLCPP_DEBUG_THROTTLE(get_logger(),*(clock_),5000,"robot continue move!");
  }
  if (collision_time >= 0.0) {
    // If collision will occurr, reduce robot speed
    const double change_ratio = collision_time / polygon->getTimeBeforeCollision();
    const Velocity safe_vel = velocity * change_ratio;
    // Check that currently calculated velocity is safer than
    // chosen for previous shapes one
    if (safe_vel < robot_action.req_vel) {
      robot_action.action_type = APPROACH;
      robot_action.req_vel = safe_vel;
      return true;
    }
  }
  

  return false;
}

void CollisionMonitor::printAction(
  const Action & robot_action, const std::shared_ptr<Polygon> action_polygon) const
{
  if (robot_action.action_type == STOP) {
    RCLCPP_DEBUG_THROTTLE(get_logger(),*(clock_),5000,
      "Robot to stop due to %s polygon",
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == SLOWDOWN) {
    RCLCPP_DEBUG_THROTTLE(get_logger(),*(clock_),5000,
      "Robot to slowdown for %f percents due to %s polygon",
      action_polygon->getSlowdownRatio() * 100,
      action_polygon->getName().c_str());
  } else if (robot_action.action_type == APPROACH) {
    RCLCPP_DEBUG_THROTTLE(get_logger(),*(clock_),5000,
      "Robot to approach for %f seconds away from collision",
      action_polygon->getTimeBeforeCollision());
  } else {  // robot_action.action_type == DO_NOTHING
    RCLCPP_DEBUG_THROTTLE(get_logger(),*(clock_),5000,
      "Robot to continue normal operation");
  }
}

void CollisionMonitor::publishPolygons() const
{
  for (std::shared_ptr<Polygon> polygon : polygons_) {
    polygon->publish();
  }
}

}  // namespace nav2_collision_monitor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_collision_monitor::CollisionMonitor)
