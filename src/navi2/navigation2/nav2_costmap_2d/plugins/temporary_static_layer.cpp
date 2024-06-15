/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2015, Fetch Robotics, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

#include "nav2_costmap_2d/temporary_static_layer.hpp"

#include <algorithm>
#include <string>

#include "pluginlib/class_list_macros.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::TemporaryStaticLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;

namespace nav2_costmap_2d
{

TemporaryStaticLayer::TemporaryStaticLayer()
: map_buffer_(nullptr)
{
}

TemporaryStaticLayer::~TemporaryStaticLayer()
{
}

void
TemporaryStaticLayer::onInitialize()
{
  global_frame_ = layered_costmap_->getGlobalFrameID();

  getParameters();
  current_ = true;
  RCLCPP_INFO(
    logger_,
    "Subscribing to the map topic (%s)",
    map_topic_.c_str());

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }
  map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic_, rclcpp::SystemDefaultsQoS(),
    std::bind(&TemporaryStaticLayer::incomingMap, this, std::placeholders::_1));
  
  mark_temp_layer_service_ = node->create_service<std_srvs::srv::SetBool>(
    "/mark_tmp_obstacle",
    std::bind(
      &TemporaryStaticLayer::enableCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  enb_mark_ = false;    

}

void
TemporaryStaticLayer::activate()
{
}

void
TemporaryStaticLayer::deactivate()
{
}

void TemporaryStaticLayer::enableCallback(
  const std::shared_ptr<rmw_request_id_t> /*request_header*/,
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  RCLCPP_ERROR(logger_,
      "Mark tmp obstacle server in!!");
  enb_mark_ = request->data;
  response->success = true; 
  if (enb_mark_) {
    response->message = "mark";
    mark_start_time_ = steady_clock_.now();
  } else {
    response->message = "unmark";
  }
  RCLCPP_ERROR(logger_,
    "Mark tmp obstacle server out!!enb_mark_ : %d.", enb_mark_); 
 
}

void
TemporaryStaticLayer::reset()
{
  has_updated_data_ = true;
  enb_mark_ = false;
  if (!map_received_) {
    return;
  }
  new_map_ = clearMap(new_map_);
  updateMap(new_map_);
  RCLCPP_ERROR(logger_,
    "enb_mark_ : %d, new_map_ size : %ld.", enb_mark_, new_map_.data.size()); 
}

void
TemporaryStaticLayer::getParameters()
{
  int temp_lethal_threshold = 0;
  double temp_tf_tol = 0.0;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_subscribe_transient_local", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
  declareParameter("map_topic", rclcpp::ParameterValue(""));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "subscribe_to_updates", subscribe_to_updates_);
  std::string private_map_topic, global_map_topic;
  node->get_parameter(name_ + "." + "map_topic", private_map_topic);
  node->get_parameter("map_topic", global_map_topic);
  if (!private_map_topic.empty()) {
    map_topic_ = private_map_topic;
  } else {
    map_topic_ = global_map_topic;
  }
  node->get_parameter("lethal_cost_threshold", temp_lethal_threshold);

  // Enforce bounds
  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 100), 0);
  map_received_ = false;
  update_in_progress_.store(false);

  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);
}

void
TemporaryStaticLayer::processMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  RCLCPP_DEBUG(logger_, "TemporaryStaticLayer: Process map");

  unsigned int size_x = new_map.info.width;
  unsigned int size_y = new_map.info.height;

  double origin_x = new_map.info.origin.position.x;
  double origin_y = new_map.info.origin.position.y;
  double resolution = new_map.info.resolution;

  RCLCPP_WARN_THROTTLE(
    logger_, *(clock_), 1000,
    "TemporaryStaticLayer: Received a %d X %d map at %f m/pix, origin(x: %f, y: %f).", size_x, size_y,
    resolution, origin_x, origin_y);

  // resize costmap if size, resolution or origin do not match
  // Costmap2D * master = layered_costmap_->getCostmap();
  if (!layered_costmap_->isRolling() && (size_x_ != size_x ||
    size_y_ != size_y ||
    resolution_ != resolution ||
    origin_x_ != origin_x ||
    origin_y_ != origin_y))
  {
    // Update the size of the layered costmap (and all layers, including this one)
    RCLCPP_INFO(
      logger_,
      "TemporaryStaticLayer: Resizing costmap to %d X %d at %f m/pix", size_x, size_y,
      resolution);
    resizeMap(
      size_x_, size_y_, resolution, origin_x, origin_y);
  } else if (size_x_ != size_x || size_y_ != size_y ||  // NOLINT
    resolution_ != resolution ||
    origin_x_ != origin_x ||
    origin_y_ != origin_y)
  {
    // only update the size of the costmap stored locally in this layer
    RCLCPP_INFO(
      logger_,
      "TemporaryStaticLayer: Resizing static layer to %d X %d at %f m/pix", size_x, size_y,
      new_map.info.resolution);
    // TodO support local costmap
    // resizeMap(
    //   size_x, size_y, new_map.info.resolution,
    //   new_map.info.origin.position.x, new_map.info.origin.position.y);
  }
  insertMap(new_map);
  map_frame_ = new_map.header.frame_id;
  new_map_ = new_map;

  x_ = y_ = 0;
  width_ = size_x_;
  height_ = size_y_;
  has_updated_data_ = true;

  current_ = true;
}

void
TemporaryStaticLayer::matchSize()
{
  // If we are using rolling costmap, the static map size is
  //   unrelated to the size of the layered costmap
  if (!layered_costmap_->isRolling()) {
    Costmap2D * master = layered_costmap_->getCostmap();
    resizeMap(
      master->getSizeInCellsX(), master->getSizeInCellsY(), master->getResolution(),
      master->getOriginX(), master->getOriginY());
  }
}

unsigned char
TemporaryStaticLayer::interpretValue(unsigned char value)
{
  // check if the static value is above the unknown or lethal thresholds
  if (track_unknown_space_ && value == unknown_cost_value_) {
    return NO_INFORMATION;
  } else if (!track_unknown_space_ && value == unknown_cost_value_) {
    return FREE_SPACE;
  } else if (value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  } else if (trinary_costmap_) {
    return FREE_SPACE;
  }

  double scale = static_cast<double>(value) / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void
TemporaryStaticLayer::insertMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  if (new_map.header.frame_id != map_frame_) {
    RCLCPP_WARN(
      logger_,
      "StaticLayer: Map update ignored. Current map is in frame %s "
      "but update was in frame %s",
      map_frame_.c_str(), new_map.header.frame_id.c_str());
  }

  unsigned int size_x = new_map.info.width;
  unsigned int size_y = new_map.info.height;
  double origin_x = new_map.info.origin.position.x;
  double origin_y = new_map.info.origin.position.y;
  unsigned int mx, my;

  unsigned int index = 0;
  // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // RCLCPP_WARN(
  //   logger_,
  //   "TemporaryStaticLayer: Received a %d X %d map, origin(x: %f, y: %f).", size_x, size_y,
  //   origin_x, origin_y);
  if (!layered_costmap_->isRolling()) {
    Costmap2D* master = layered_costmap_->getCostmap();
    if (master->worldToMap(origin_x, origin_y, mx, my)) {
      // RCLCPP_INFO_THROTTLE(
      //   logger_, *(clock_), 3000,
      //   "origer image index(mx : %d, my : %d).", static_cast<int>(mx), static_cast<int>(my));
      for (unsigned int y = 0; y < size_y; y++) {        
        for (unsigned int x = 0; x < size_x; x++) {
          unsigned char value = new_map.data[index];
          if (value == lethal_threshold_) {
            // RCLCPP_INFO(logger_, "value : %d.", static_cast<int>(value));
            // costmap_[index] = LETHAL_OBSTACLE;
            master->setCost(x + mx, y + my, LETHAL_OBSTACLE);
          }
          ++index;
        }
      }
    }
  } else {
    //Todo support localcostmap
    RCLCPP_WARN(
      logger_,
      "Can`t support now!!!");
  }

}

nav_msgs::msg::OccupancyGrid
TemporaryStaticLayer::clearMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  nav_msgs::msg::OccupancyGrid temp_map = new_map;
  if (temp_map.header.frame_id != map_frame_) {
    RCLCPP_WARN(
      logger_,
      "StaticLayer: Map update ignored. Current map is in frame %s "
      "but update was in frame %s",
      map_frame_.c_str(), temp_map.header.frame_id.c_str());
  }

  unsigned int size_x = temp_map.info.width;
  unsigned int size_y = temp_map.info.height;
  double origin_x = temp_map.info.origin.position.x;
  double origin_y = temp_map.info.origin.position.y;
  unsigned int mx, my;

  unsigned int index = 0;
  unsigned char unknown_cost_value = 255;
  // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (!layered_costmap_->isRolling()) {
    Costmap2D* master = layered_costmap_->getCostmap();
    if (master->worldToMap(origin_x, origin_y, mx, my)) {
      RCLCPP_INFO(logger_, "origin index in image frame :(mx : %d, my : %d)", static_cast<int>(mx), static_cast<int>(my));
      for (unsigned int y = 0; y < size_y; y++) {        
        for (unsigned int x = 0; x < size_x; x++) {
          temp_map.data[index] = unknown_cost_value;
          ++index;
        }
      }
    }
  } else {
    //Todo support localcostmap
    RCLCPP_WARN(
      logger_,
      "Can`t support now!!!");
  }
  // temp_map.data.clear();
  return temp_map;

}

void
TemporaryStaticLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
{
  if (!enb_mark_) {
    return;
  }

  updateMap(*new_map);
  // std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  // if (!map_received_) {
  //   RCLCPP_INFO(logger_, "No map received before this time.");
  //   processMap(*new_map);
  //   RCLCPP_INFO(logger_, "Have processed the first received map, now set map_received_=true.");
  //   map_received_ = true;
  // }
  // if (update_in_progress_.load()) {
  //   map_buffer_ = new_map;
  // } else {
  //   processMap(*new_map);
  //   map_buffer_ = nullptr;
  // }
  auto during_time = steady_clock_.now() - mark_start_time_;
  auto during_time_seconds = during_time.seconds();
  RCLCPP_WARN(
    logger_,
    "Marking : %.2f s", during_time_seconds); 
  // When triggered asynchronously, prevent the addition from being delayed.
  if (during_time_seconds > 1.0) {
    enb_mark_ = false;
  }
}

void 
TemporaryStaticLayer::updateMap(const nav_msgs::msg::OccupancyGrid & new_map)
{
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!map_received_) {
    RCLCPP_INFO(logger_, "No map received before this time.");
    processMap(new_map);
    RCLCPP_INFO(logger_, "Have processed the first received map, now set map_received_=true.");
    map_received_ = true;
  }
  if (update_in_progress_.load()) {
    // RCLCPP_ERROR(
    //   logger_,
    //   "!!!!!!!!!!!!!!!");
    // nav_msgs::msg::OccupancyGrid temp_map = new_map;
    // *map_buffer_ = temp_map;
    return;
  } else {
    processMap(new_map);
    map_buffer_ = nullptr;
  }

}

void
TemporaryStaticLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y,
  double * max_x,
  double * max_y)
{
  if (!map_received_) {
    return;
  }

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  update_in_progress_.store(true);

  // If there is a new available map, load it.
  if (map_buffer_) {
    processMap(*map_buffer_);
    map_buffer_ = nullptr;
  }

  if (!layered_costmap_->isRolling() ) {
    if (!(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  double wx, wy;

  mapToWorld(x_, y_, wx, wy);
  *min_x = std::min(wx, *min_x);
  *min_y = std::min(wy, *min_y);

  mapToWorld(x_ + width_, y_ + height_, wx, wy);
  *max_x = std::max(wx, *max_x);
  *max_y = std::max(wy, *max_y);

  has_updated_data_ = false;
}

void
TemporaryStaticLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    update_in_progress_.store(false);
    RCLCPP_ERROR(logger_,
      "enabled_ : %d.", enabled_); 
    return;
  }
  if (!map_received_) {
    update_in_progress_.store(false);
    return;
  }

  if (!layered_costmap_->isRolling()) {
    insertMap(new_map_);
  } else {
    // If rolling window, the master_grid is unlikely to have same coordinates as this layer
    unsigned int mx, my;
    double wx, wy;
    // Might even be in a different frame
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_->lookupTransform(
        map_frame_, global_frame_, tf2::TimePointZero,
        transform_tolerance_);
    } catch (tf2::TransformException & ex) {
      RCLCPP_ERROR(logger_, "TemporaryStaticLayer: %s", ex.what());
      update_in_progress_.store(false);
      return;
    }
    // Copy map data given proper transformations
    tf2::Transform tf2_transform;
    tf2::fromMsg(transform.transform, tf2_transform);

    for (int i = min_i; i < max_i; ++i) {
      for (int j = min_j; j < max_j; ++j) {
        // Convert master_grid coordinates (i,j) into global_frame_(wx,wy) coordinates
        layered_costmap_->getCostmap()->mapToWorld(i, j, wx, wy);
        // Transform from global_frame_ to map_frame_
        tf2::Vector3 p(wx, wy, 0);
        p = tf2_transform * p;
        // Set master_grid with cell from map
        if (worldToMap(p.x(), p.y(), mx, my)) {
          if (!use_maximum_) {
            master_grid.setCost(i, j, getCost(mx, my));
          } else {
            master_grid.setCost(i, j, std::max(getCost(mx, my), master_grid.getCost(i, j)));
          }
        }
      }
    }
  }
  update_in_progress_.store(false);
  current_ = true;
}

}  // namespace nav2_costmap_2d
