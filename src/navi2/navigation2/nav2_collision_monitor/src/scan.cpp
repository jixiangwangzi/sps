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

#include "nav2_collision_monitor/scan.hpp"

#include <cmath>
#include <functional>
#include "nav2_util/node_utils.hpp"
namespace nav2_collision_monitor
{

Scan::Scan(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & source_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const std::string & global_frame_id,
  const tf2::Duration & transform_tolerance,
  const rclcpp::Duration & source_timeout)
: Source(
    node, source_name, tf_buffer, base_frame_id, global_frame_id,
    transform_tolerance, source_timeout),
  data_(nullptr)
{
  RCLCPP_INFO(logger_, "[%s]: Creating Scan", source_name_.c_str());
}

Scan::~Scan()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Scan", source_name_.c_str());
  data_sub_.reset();
}

void Scan::configure()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  std::string source_topic;

  // Laser scanner has no own parameters
  getParameters(source_topic);
  //getCommonParameters(source_topic);

  rclcpp::QoS scan_qos = rclcpp::SensorDataQoS();  // set to default
  data_sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
    source_topic, scan_qos,
    std::bind(&Scan::dataCallback, this, std::placeholders::_1));
}

void Scan::getParameters(std::string & source_topic)
{
  auto node = node_.lock();
  if(!node){
    throw std::runtime_error{"failed to lock node"};
  }

  getCommonParameters(source_topic);
  
  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".min_obstacle_range", rclcpp::ParameterValue(0.0));
  min_obstacle_range_ = node->get_parameter(source_name_ + ".min_obstacle_range").as_double();

  nav2_util::declare_parameter_if_not_declared(node, source_name_ + ".max_obstacle_range", rclcpp::ParameterValue(4.0));
  max_obstacle_range_ = node->get_parameter(source_name_ + ".max_obstacle_range").as_double();

  RCLCPP_INFO(logger_,"collision_monitor %s parameter----max_obstacle_range:            %f",source_topic.c_str(),max_obstacle_range_);
  RCLCPP_INFO(logger_,"collision_monitor %s parameter----min_obstacle_range:            %f",source_topic.c_str(),min_obstacle_range_);

}

void Scan::getData(
  const rclcpp::Time & curr_time,
  std::vector<Point> & data) const
{
  // Ignore data from the source if it is not being published yet or
  // not being published for a long time
  if (data_ == nullptr) {
    return;
  }
  if (!sourceValid(data_->header.stamp, curr_time)) {
    return;
  }

  // Obtaining the transform to get data from source frame and time where it was received
  // to the base frame and current time
  tf2::Transform tf_transform;
  if (!getTransform(data_->header.frame_id, data_->header.stamp, curr_time, tf_transform)) {
    return;
  }

  // Calculate poses and refill data array
  float angle = data_->angle_min;
  for (size_t i = 0; i < data_->ranges.size(); i++) {
    if (data_->ranges[i] >= data_->range_min && data_->ranges[i] <= data_->range_max && data_->ranges[i] >= min_obstacle_range_ && data_->ranges[i] <= max_obstacle_range_) {
      // Transform point coordinates from source frame -> to base frame
      tf2::Vector3 p_v3_s(
        data_->ranges[i] * std::cos(angle),
        data_->ranges[i] * std::sin(angle),
        0.0);
      tf2::Vector3 p_v3_b = tf_transform * p_v3_s;

      // Refill data array
      data.push_back({p_v3_b.x(), p_v3_b.y()});
    }
    angle += data_->angle_increment;
  }
}

void Scan::dataCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr msg)
{
  data_ = msg;
}

}  // namespace nav2_collision_monitor
