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

#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H

#include "cartographer/common/time.h"
#include <rclcpp/rclcpp.hpp>

namespace cartographer_ros {

rclcpp::Time ToRos(::cartographer::common::Time time);

::cartographer::common::Time FromRos(const rclcpp::Time& time);

}  // namespace cartographer_ros

#endif  // CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_TIME_CONVERSION_H
