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
#ifndef CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_MAIN_H
#define CARTOGRAPHER_ROS_CARTOGRAPHER_ROS_NODE_MAIN_H

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
namespace cartographer_ros {
void Run(const std::string& configuration_directory, const std::string& configuration_basename, 
    const bool& collect_metrics, const std::string& load_state_filename, const bool& load_frozen_state,
    const bool& start_trajectory_with_default_topics){
    constexpr double kTfBufferCacheTimeInSeconds = 10.;
    tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
    tf2_ros::TransformListener tf(tf_buffer);
    NodeOptions node_options;
    TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
        LoadOptions(configuration_directory, configuration_basename);
    auto map_builder =
        cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
    // Node node(node_options, std::move(map_builder), &tf_buffer,
    //             FLAGS_collect_metrics);
    Node node(node_options, std::move(map_builder), &tf_buffer,
                collect_metrics);
    if (!load_state_filename.empty()) {
        node.LoadState(load_state_filename, load_frozen_state);
    }

    if (start_trajectory_with_default_topics) {
        node.StartTrajectoryWithDefaultTopics(trajectory_options);
    }
    ros::spin();
    
    return ;

}
}
#endif