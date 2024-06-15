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

#include "nav2_collision_monitor/circle.hpp"

#include <math.h>
#include <cmath>
#include <exception>

#include "nav2_util/node_utils.hpp"

namespace nav2_collision_monitor
{

Circle::Circle(
  const nav2_util::LifecycleNode::WeakPtr & node,
  const std::string & polygon_name,
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::string & base_frame_id,
  const tf2::Duration & transform_tolerance)
: Polygon::Polygon(node, polygon_name, tf_buffer, base_frame_id, transform_tolerance)
{
  initialize_ = false;
  RCLCPP_INFO(logger_, "[%s]: Creating Circle", polygon_name_.c_str());
}

Circle::~Circle()
{
  RCLCPP_INFO(logger_, "[%s]: Destroying Circle", polygon_name_.c_str());
}

void Circle::getPolygon(std::vector<Point> & poly) const
{
  // Number of polygon points. More edges means better approximation.
  const double polygon_edges = 20;//16
  // Increment of angle during points position calculation
  double angle_increment = 2 * M_PI / polygon_edges;

  // Clear polygon before filling
  poly.clear();
  //polygon_.points.clear();

  // Making new polygon looks like a circle
  Point p;
  //geometry_msgs::msg::Point32 p_s;
  for (double angle = 0.0; angle < 2 * M_PI; angle += angle_increment) {
    p.x = radius_ * std::cos(angle);
    p.y = radius_ * std::sin(angle);
    if(p.y > 0.4){
      p.y = 0.4;   //TODO:configure  circle
    }
    if(p.y < -0.4){
      p.y = -0.4;
    }
    if(p.x < -0.4){
      p.x = -0.4;
    }
    poly.push_back(p);
    //p_s.x = p.x;
    //p_s.y = p.y;
    //polygon_.points.push_back(p_s);
  }
}

// TODO:  modify 
int Circle::getPointsInside(const std::vector<Point> & points) const
{
  int num = 0;
  for (Point point : points) {
    if (point.x * point.x + point.y * point.y < radius_squared_) { 
      num++;
    }
  }

  return num;
}

void Circle::updatePolygonStop(const Velocity & velocity,const double max_vel)
{
  RCLCPP_INFO(rclcpp::get_logger("circle"),"cirecle update polygonstop!");
  polygon_.points.clear();
  Point p;
  geometry_msgs::msg::Point32 p_s;
  const double polygon_edges = 20;
  double angle_increment = 2 * M_PI / polygon_edges;
  double radius;
  if(velocity.x <= 0.05){    
    radius = 0.28;
  }else{
    double ratio = velocity.x / 0.7;
    radius = ratio * radius_;
    if(radius < 0.28){
      radius = 0.28;
    }
  }
  for(double angle = 0.0; angle < 2 * M_PI; angle += angle_increment){
    p.x = radius * std::cos(angle);
    p.y = radius * std::sin(angle);
    if(p.y > 0.3){
      p.y = 0.3;   //TODO:configure
    }
    if(p.y < -0.3){
      p.y = -0.3;
    }
    if(p.x < -0.3){
      p.x = -0.3;
    }
    p_s.x = p.x;
    p_s.y = p.y;
    polygon_.points.push_back(p_s);
  }   
}

void Circle::updatePolygonSlowdown(const Velocity & velocity,const double max_vel)
{
  RCLCPP_INFO(rclcpp::get_logger("circle"),"cirecle update polygonslowdown");
  polygon_.points.clear();
  Point p;
  geometry_msgs::msg::Point32 p_s;
  const double polygon_edges = 20;
  double angle_increment = 2 * M_PI / polygon_edges;
  double radius;
  if(velocity.x == 0.0){    
    radius = 0.4;
  }else{
    double ratio = velocity.x / 0.7;
    radius = ratio * radius_;
    if(radius < 0.4){
      radius = 0.45;
    }
  }
  for(double angle = 0.0; angle < 2 * M_PI; angle += angle_increment){
    p.x = radius * std::cos(angle);
    p.y = radius * std::sin(angle);
    if(p.y > 0.4){
      p.y = 0.4;   //TODO:configure
    }
    if(p.y < -0.4){
      p.y = -0.4;
    }
    if(p.x < -0.3){
      p.x = -0.3;
    }
    p_s.x = p.x;
    p_s.y = p.y;
    polygon_.points.push_back(p_s);
  }  
}

void Circle::publish() const
{
  // if (!visualize_) {
  //   return;
  // }

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  // Fill PolygonStamped struct
  std::unique_ptr<geometry_msgs::msg::PolygonStamped> poly_s =
    std::make_unique<geometry_msgs::msg::PolygonStamped>();
  poly_s->header.stamp = node->now();
  poly_s->header.frame_id = base_frame_id_;
  poly_s->polygon = polygon_;

  // Publish polygon
  polygon_pub_->publish(std::move(poly_s));
}

bool Circle::getParameters(std::string & polygon_pub_topic, std::string & footprint_topic)
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  if (!getCommonParameters(polygon_pub_topic)) {
    return false;
  }

  // There is no footprint subscription for the Circle. Thus, set string as empty.
  footprint_topic.clear();

  try {
    // Leave it not initialized: the will cause an error if it will not set
    nav2_util::declare_parameter_if_not_declared(
      node, polygon_name_ + ".radius", rclcpp::PARAMETER_DOUBLE);
    radius_ = node->get_parameter(polygon_name_ + ".radius").as_double();
    radius_squared_ = radius_ * radius_;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      logger_,
      "[%s]: Error while getting circle parameters: %s",
      polygon_name_.c_str(), ex.what());
    return false;
  }

  return true;
}

}  // namespace nav2_collision_monitor
