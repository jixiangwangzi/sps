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

#include "cartographer_ros/node_main.h"



// void cartographer_ros::Run() {
//   constexpr double kTfBufferCacheTimeInSeconds = 10.;
//   tf2_ros::Buffer tf_buffer{::ros::Duration(kTfBufferCacheTimeInSeconds)};
//   tf2_ros::TransformListener tf(tf_buffer);
//   NodeOptions node_options;
//   TrajectoryOptions trajectory_options;
//   std::tie(node_options, trajectory_options) =
//       LoadOptions(FLAGS_configuration_directory, FLAGS_configuration_basename);

//   auto map_builder =
//       cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);
//   Node node(node_options, std::move(map_builder), &tf_buffer,
//             FLAGS_collect_metrics);
//   if (!FLAGS_load_state_filename.empty()) {
//     node.LoadState(FLAGS_load_state_filename, FLAGS_load_frozen_state);
//   }

//   if (FLAGS_start_trajectory_with_default_topics) {
//     node.StartTrajectoryWithDefaultTopics(trajectory_options);
//   }
//   // ::ros::spin();

//   // node.FinishAllTrajectories();
//   // node.RunFinalOptimization();

//   // if (!FLAGS_save_state_filename.empty()) {
//   //   node.SerializeState(FLAGS_save_state_filename,
//   //                       true /* include_unfinished_submaps */);
//   // }
// }


int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  // CHECK(!FLAGS_configuration_directory.empty())
  //     << "-configuration_directory is missing.";
  // CHECK(!FLAGS_configuration_basename.empty())
  //     << "-configuration_basename is missing.";

  ::ros::init(argc, argv, "cartographer_node");
  ::ros::start();

  // cartographer_ros::ScopedRosLogSink ros_log_sink;
  // cartographer_ros::Run();
  ::ros::shutdown();
}
