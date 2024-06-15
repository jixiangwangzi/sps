// Copyright (c) 2018 Intel Corporation
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

#include <vector>
#include <string>
#include <algorithm>
#include <memory>

#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/clear_costmap_service.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_costmap_2d
{

using std::vector;
using std::string;
using std::shared_ptr;
using std::any_of;
using ClearExceptRegion = nav2_msgs::srv::ClearCostmapExceptRegion;
using ClearAroundRobot = nav2_msgs::srv::ClearCostmapAroundRobot;
using ClearEntirely = nav2_msgs::srv::ClearEntireCostmap;

ClearCostmapService::ClearCostmapService(
  const nav2_util::LifecycleNode::WeakPtr & parent,
  Costmap2DROS & costmap)
: costmap_(costmap)
{
  auto node = parent.lock();
  logger_ = node->get_logger();
  reset_value_ = costmap_.getCostmap()->getDefaultValue();

  std::vector<std::string> clearable_layers{"obstacle_layer", "voxel_layer", "range_layer"};
  nav2_util::declare_parameter_if_not_declared(node, "clearable_layers", rclcpp::ParameterValue(clearable_layers));
  
  node->get_parameter("clearable_layers", clearable_layers);
  for(size_t i = 0; i < clearable_layers.size(); i++) {
      RCLCPP_WARN(
        logger_, ("Recovery behavior will clear layer " + clearable_layers[i]).c_str() );
      clearable_layers_.insert(clearable_layers[i]);
  }

  clear_except_service_ = node->create_service<ClearExceptRegion>(
    "clear_except_" + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearExceptRegionCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  clear_around_service_ = node->create_service<ClearAroundRobot>(
    "clear_around_" + costmap.getName(),
    std::bind(
      &ClearCostmapService::clearAroundRobotCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  clear_entire_service_ = node->create_service<ClearEntirely>(
    "clear_entirely_" + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearEntireCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

  clear_appointed_service_ = node->create_service<ClearEntirely>(
    "clear_appointed_" + costmap_.getName(),
    std::bind(
      &ClearCostmapService::clearAppointedCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
}

void ClearCostmapService::clearExceptRegionCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearExceptRegion::Request> request,
  const shared_ptr<ClearExceptRegion::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_,
    ("Received request to clear except a region the " + costmap_.getName()).c_str());

  clearRegion(request->reset_distance, true);
}

void ClearCostmapService::clearAroundRobotCallback(
  const shared_ptr<rmw_request_id_t>/*request_header*/,
  const shared_ptr<ClearAroundRobot::Request> request,
  const shared_ptr<ClearAroundRobot::Response>/*response*/)
{
  clearRegion(request->reset_distance, false);
}

void ClearCostmapService::clearEntireCallback(
  const std::shared_ptr<rmw_request_id_t>/*request_header*/,
  const std::shared_ptr<ClearEntirely::Request>/*request*/,
  const std::shared_ptr<ClearEntirely::Response>/*response*/)
{
  RCLCPP_INFO(
    logger_,
    ("Received request to clear entirely the " + costmap_.getName()).c_str());

  auto start_time = steady_clock_.now();
  clearEntirely();
  auto end_time = steady_clock_.now() - start_time;
  RCLCPP_WARN(
    logger_, "Clear costmap used %.6f s", end_time.seconds()); 
}

void ClearCostmapService::clearAppointedCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<ClearEntirely::Request> /*request*/,
  const std::shared_ptr<ClearEntirely::Response> /*response*/)
{
  auto start_time = steady_clock_.now();
  clearAppointedLayers();
  auto end_time = steady_clock_.now() - start_time;
  RCLCPP_WARN(
    logger_, "Clear costmap used %.6f s", end_time.seconds()); 
}

void ClearCostmapService::clearRegion(const double reset_distance, bool invert)
{
  double x, y;

  if (!getPosition(x, y)) {
    RCLCPP_ERROR(
      logger_,
      "Cannot clear map because robot pose cannot be retrieved.");
    return;
  }

  static auto layers = costmap_.getLayeredCostmap()->getPlugins();

  for (auto & layer : *layers) {
    if (layer->isClearable()) {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      clearLayerRegion(costmap_layer, x, y, reset_distance, invert);
    }
  }
}

void ClearCostmapService::clearAppointedLayers()
{
  // double x, y;

  // if (!getPosition(x, y)) {
  //   RCLCPP_ERROR(
  //     logger_,
  //     "Cannot clear map because robot pose cannot be retrieved.");
  //   return;
  // }

  auto layers = costmap_.getLayeredCostmap()->getPlugins();
  for (auto & layer : *layers) {
    std::string name = layer->getName();
    // RCLCPP_INFO(
    //   logger_, ("Have " + name).c_str());
    // auto slash = name.rfind('/');
    // if( slash != std::string::npos ){
    //     name = name.substr(slash+1);
    // }

    if(clearable_layers_.count(name)!=0) {
      auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      // RCLCPP_WARN(
      //   logger_,
      //   ("Sart clear " + costmap_layer->getName()).c_str());
      costmap_layer->reset();  
      // if (layer->isClearable()) {
      //   auto costmap_layer = std::static_pointer_cast<CostmapLayer>(layer);
      //   RCLCPP_WARN(
      //     logger_,
      //     ("Sart clear " + costmap_layer->getName()).c_str());
      //   // clearLayerRegion(costmap_layer, x, y, reset_distance, invert);
      //   costmap_layer->reset();
      // }
    }
  }
}

void ClearCostmapService::clearLayerRegion(
  shared_ptr<CostmapLayer> & costmap, double pose_x, double pose_y, double reset_distance,
  bool invert)
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  double start_point_x = pose_x - reset_distance / 2;
  double start_point_y = pose_y - reset_distance / 2;
  double end_point_x = start_point_x + reset_distance;
  double end_point_y = start_point_y + reset_distance;

  int start_x, start_y, end_x, end_y;
  costmap->worldToMapEnforceBounds(start_point_x, start_point_y, start_x, start_y);
  costmap->worldToMapEnforceBounds(end_point_x, end_point_y, end_x, end_y);

  costmap->clearArea(start_x, start_y, end_x, end_y, invert);

  double ox = costmap->getOriginX(), oy = costmap->getOriginY();
  double width = costmap->getSizeInMetersX(), height = costmap->getSizeInMetersY();
  costmap->addExtraBounds(ox, oy, ox + width, oy + height);
}

void ClearCostmapService::clearEntirely()
{
  std::unique_lock<Costmap2D::mutex_t> lock(*(costmap_.getCostmap()->getMutex()));
  costmap_.resetLayers();
}

bool ClearCostmapService::getPosition(double & x, double & y) const
{
  geometry_msgs::msg::PoseStamped pose;
  if (!costmap_.getRobotPose(pose)) {
    return false;
  }

  x = pose.pose.position.x;
  y = pose.pose.position.y;

  return true;
}

}  // namespace nav2_costmap_2d
