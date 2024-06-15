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

#include "cartographer_ros/map_builder_bridge.h"

#include "absl/memory/memory.h"
#include "absl/strings/str_cat.h"
#include "cartographer/io/color.h"
#include "cartographer/io/proto_stream.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include <string>

namespace cartographer_ros {
namespace {

using ::cartographer::transform::Rigid3d;

constexpr double kTrajectoryLineStripMarkerScale = 0.07;
constexpr double kLandmarkMarkerScale = 0.4;
constexpr double kConstraintMarkerScale = 0.025;

::std_msgs::msg::ColorRGBA ToMessage(const cartographer::io::FloatColor& color) {
  ::std_msgs::msg::ColorRGBA result;
  result.r = color[0];
  result.g = color[1];
  result.b = color[2];
  result.a = 1.f;
  return result;
}

visualization_msgs::msg::Marker CreateTrajectoryMarker(const int trajectory_id,
                                                  const std::string& frame_id,
                                                  rclcpp::Time node_time) {
  visualization_msgs::msg::Marker marker;
  marker.ns = absl::StrCat("Trajectory ", trajectory_id);
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.header.stamp = node_time;
  marker.header.frame_id = frame_id;
  marker.color = ToMessage(cartographer::io::GetColor(trajectory_id));
  marker.scale.x = kTrajectoryLineStripMarkerScale;
  marker.pose.orientation.w = 1.;
  marker.pose.position.z = 0.05;
  return marker;
}

int GetLandmarkIndex(
    const std::string& landmark_id,
    std::unordered_map<std::string, int>* landmark_id_to_index) {
  auto it = landmark_id_to_index->find(landmark_id);
  if (it == landmark_id_to_index->end()) {
    const int new_index = landmark_id_to_index->size();
    landmark_id_to_index->emplace(landmark_id, new_index);
    return new_index;
  }
  return it->second;
}

visualization_msgs::msg::Marker CreateLandmarkMarker(const std::string& landmark_index,
                                                const Rigid3d& landmark_pose,
                                                const std::string& frame_id,
                                                rclcpp::Time node_time) {
  visualization_msgs::msg::Marker marker;
  marker.ns = "Landmarks";
  marker.id = atoi(landmark_index.c_str());
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.header.stamp = node_time;
  marker.header.frame_id = frame_id;
  marker.scale.x = kLandmarkMarkerScale;
  marker.scale.y = kLandmarkMarkerScale;
  marker.scale.z = kLandmarkMarkerScale;
  marker.color = ToMessage(cartographer::io::GetColor(atoi(landmark_index.c_str())));
  marker.pose = ToGeometryMsgPose(landmark_pose);
  marker.text = landmark_index;
  return marker;
}

void PushAndResetLineMarker(visualization_msgs::msg::Marker* marker,
                            std::vector<visualization_msgs::msg::Marker>* markers) {
  markers->push_back(*marker);
  ++marker->id;
  marker->points.clear();
}

}  // namespace

MapBuilderBridge::MapBuilderBridge(
    const NodeOptions& node_options,
    std::unique_ptr<cartographer::mapping::MapBuilderInterface> map_builder,
    tf2_ros::Buffer* tf_buffer, std::shared_ptr<cartographer::GlobalMessage>& globalMessagePtr,
    std::shared_ptr<rclcpp::Node> node_ptr)
    : node_options_(node_options),
      map_builder_(std::move(map_builder)),
      tf_buffer_(tf_buffer),
      scorePoseFromSubmapPtr_(nullptr), scorePoseFromMapPtr_(nullptr),
      globalMessagePtr_(globalMessagePtr), node_ptr_(node_ptr) {
        map_builder_->setGlobalMessagePtr(globalMessagePtr_);
}

void MapBuilderBridge::LoadState(const std::string& state_filename,
                                 bool load_frozen_state) {
  // Check if suffix of the state file is ".pbstream".
  const std::string suffix = ".pbstream";
  CHECK_EQ(state_filename.substr(
               std::max<int>(state_filename.size() - suffix.size(), 0)),
           suffix)
      << "The file containing the state to be loaded must be a "
         ".pbstream file.";
  LOG(INFO) << "Loading saved state '" << state_filename << "'...";
  cartographer::io::ProtoStreamReader stream(state_filename);
  map_builder_->LoadState(&stream, load_frozen_state);
}

// int MapBuilderBridge::LoadMap(const std::string map_path, std::shared_ptr<nav_msgs::OccupancyGrid>& occ_map){
//   std::string image_file = map_path + "/mymap.pgm";
//    img;
//   double resolution_ = 0.05;
//   // double threshold_free_ = 0.196;
//   // double threshold_occupied_ = 0.65;
//   std::vector<double> origin_ = {-9.97089, -7.97386, 0.0};
//   // std::vector<double> origin_ = {-8.9, -7.94912, 0.0};
//   occ_map = std::make_shared<nav_msgs::OccupancyGrid>();
//   try {
//     img = cv::imread(image_file, CV_LOAD_IMAGE_GRAYSCALE);
//     if (img.empty()) {
//       ROS_ERROR("GingerMap: map png file is empty");
//       // rdmPub();
//       return -5;  // MAP_PNG_UNVALID
//     }
//   } catch (cv::Exception &e) {
//     ROS_ERROR("GingerMap: cant open map png file");
//     // rdmPub();
//     return -3;  // NO_MAP_PNG
//   }
//     // get maps
//   tf2::Quaternion q;
//   q.setRPY(0.0, 0.0, origin_[2]);
//   q.normalize();
//   ros::Time now = ros::Time::now();
//   occ_map->info.resolution = resolution_;
//   occ_map->info.origin.position.x = origin_[0];
//   occ_map->info.origin.position.y = origin_[1];
//   occ_map->info.origin.orientation = tf2::toMsg(q);
//   occ_map->info.width = img.cols;
//   occ_map->info.height = img.rows;
//   occ_map->data.resize(img.rows * img.cols);
//   occ_map->header.frame_id = "map";
//   occ_map->header.stamp = now;
//   Eigen::MatrixXd matrix_img(img.rows,img.cols);
//   for (int i = 0; i < img.rows; i++) {
//     for (int j = 0; j < img.cols; j++) {
//       int occ_index = j + (img.rows - i - 1) * img.cols;
//       uint8_t pixel = img.at<uint8_t>(i, j);
//       matrix_img(i,j) = pixel;
//       if (pixel < 10) {
//         occ_map->data[occ_index] = 100;
//       } else if (pixel > 180) {
//         occ_map->data[occ_index] = 0;
//       } else {
//         occ_map->data[occ_index] = -1;
//       }

//     }
//   }
//   map_builder_->AddTrajectoryForLoadMap(map_path, matrix_img);
//   return 0;
// }

bool MapBuilderBridge::FilterMap(Eigen::MatrixXd& map_img){
  Eigen::MatrixXd map_img_tmp = map_img;
  for(unsigned int i = 0; i < map_img_tmp.rows(); i++){
    for(unsigned int j = 0; j < map_img_tmp.cols(); j++){
      double min = 255;
      for(int m = -1; m <= 1; m++){
        for(int n = -1; n <=1; n++){
          if((i + m >= 0) && (i + m < map_img_tmp.rows()) 
              && (j + n >= 0) && (j + n < map_img_tmp.cols())){
            // map_img(i + m,j + n) = std::min(map_img(i + m,j + n) + 30, map_img_tmp(i, j));
            if(min > map_img_tmp(i + m,j + n)){
              min = map_img_tmp(i + m,j + n);
            }
          }
        }
      }
      if((min < 100) && (map_img(i ,j) > 100)){
        map_img(i, j) = map_img(i, j) - 50;
      }
      // map_img(i, j) = std::min(map_img(i, j), min);
    }
  }
  return true;
}

bool MapBuilderBridge::SetOccMap(const nav_msgs::msg::OccupancyGrid& occ_map){

  Eigen::Vector2f origin(occ_map.info.origin.position.x, occ_map.info.origin.position.y);
  LOG(INFO) << "map_builder_matrix_img start!!!";
  Eigen::MatrixXd matrix_img(occ_map.info.height, occ_map.info.width);
  for(unsigned int i = 0; i < occ_map.info.height; i++){
    for(unsigned int j = 0; j < occ_map.info.width; j++){
      if(occ_map.data[i * occ_map.info.width + j] == 0){
              matrix_img(occ_map.info.height - i -1, j) = 255;
      }
      else if(occ_map.data[i * occ_map.info.width + j] == 100){
          matrix_img(occ_map.info.height - i -1, j) = 0;
      }
      else{
          matrix_img(occ_map.info.height - i -1, j) = 125;
      }
    }
  }
  if(globalMessagePtr_ != nullptr && globalMessagePtr_->getFlagIsLocalization()){
    FilterMap(matrix_img);
  }

  // for(int i = 0; i < occ_map.data.size(); i++){
  //   if(occ_map.data[i] == 0){
  //     matrix_img(i/occ_map.info.width, i%occ_map.info.width) = 255;
  //   }
  //   else if(occ_map.data[i] == 100){
  //     matrix_img(i/occ_map.info.width, i%occ_map.info.width) = 0;
  //   }
  //   else{
  //     matrix_img(i/occ_map.info.width, i%occ_map.info.width) = 125;
  //   }

  // }
  LOG(INFO) << "map_builder_->AddTrajectoryForLoadMap(..) start!!!";
  map_builder_->AddTrajectoryForLoadMap(origin, matrix_img);
  LOG(INFO) << "map_builder_->AddTrajectoryForLoadMap(..) end";
  return true;
}


int MapBuilderBridge::AddTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const TrajectoryOptions& trajectory_options) {
  const int trajectory_id = map_builder_->AddTrajectoryBuilder(
      expected_sensor_ids, trajectory_options.trajectory_builder_options,
      [this](const int trajectory_id, const ::cartographer::common::Time time,
             const Rigid3d local_pose,
             ::cartographer::sensor::RangeData range_data_in_local,
             const std::unique_ptr<
                 const ::cartographer::mapping::TrajectoryBuilderInterface::
                     InsertionResult>) {
        OnLocalSlamResult(trajectory_id, time, local_pose, range_data_in_local);
      });
  scorePoseFromSubmapPtr_ = map_builder_->getScorePoseFromSubmapPtr();
  scorePoseFromMapPtr_ = map_builder_->getScorePoseFromMapPtr();
  LOG(INFO) << "Added trajectory with ID '" << trajectory_id << "'.";

  // Make sure there is no trajectory with 'trajectory_id' yet.
  CHECK_EQ(sensor_bridges_.count(trajectory_id), 0);
  sensor_bridges_[trajectory_id] = absl::make_unique<SensorBridge>(
      trajectory_options.num_subdivisions_per_laser_scan,
      trajectory_options.tracking_frame,
      node_options_.lookup_transform_timeout_sec, tf_buffer_,
      map_builder_->GetTrajectoryBuilder(trajectory_id));
  auto emplace_result =
      trajectory_options_.emplace(trajectory_id, trajectory_options);
  CHECK(emplace_result.second == true);
  return trajectory_id;
}

void MapBuilderBridge::FinishTrajectory(const int trajectory_id) {
  LOG(INFO) << "Finishing trajectory with ID '" << trajectory_id << "'...";

  // Make sure there is a trajectory with 'trajectory_id'.
  CHECK(GetTrajectoryStates().count(trajectory_id));
  map_builder_->FinishTrajectory(trajectory_id);
  sensor_bridges_.erase(trajectory_id);
}

void MapBuilderBridge::DeleteTrajectory(const int trajectory_id) {
  LOG(INFO) << "Delete trajectory with ID '" << trajectory_id << "'  start ...";
  // Make sure there is a trajectory with 'trajectory_id'.
 // CHECK(GetTrajectoryStates().count(trajectory_id));
  // LOG(INFO) << "Delete trajectory with ID '" << trajectory_id << "'  1 ...";
  map_builder_->DeleteTrajectory(trajectory_id);
  // LOG(INFO) << "Delete trajectory with ID '" << trajectory_id << "'  2 ...";
  sensor_bridges_.erase(trajectory_id);
  // LOG(INFO) << "Delete trajectory with ID '" << trajectory_id << "'  3 ...";
  trajectory_options_.erase(trajectory_id);
  LOG(INFO) << "trajectory_options_.size(): "<< trajectory_options_.size();
  for (auto it : trajectory_options_){
     LOG(INFO) << "trajectory_options_.key: : "<< it.first;
  }
  
  LOG(INFO) << "Delete trajectory with ID '" << trajectory_id << "'  end ...";
}

void MapBuilderBridge::RunFinalOptimization() {
  LOG(INFO) << "Running final trajectory optimization...";
  map_builder_->pose_graph()->RunFinalOptimization();
}

bool MapBuilderBridge::SerializeState(const std::string& filename,
                                      const bool include_unfinished_submaps) {
  return map_builder_->SerializeStateToFile(include_unfinished_submaps,
                                            filename);
}

void MapBuilderBridge::HandleSubmapQuery(
    const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response) {
  cartographer::mapping::proto::SubmapQuery::Response response_proto;
  cartographer::mapping::SubmapId submap_id{request->trajectory_id,
                                            request->submap_index};
  if((unsigned int) request->trajectory_id >= GetTrajectoryStates().size()){
    LOG(ERROR) << "MapBuilderBridge:: request->trajectory_id >= GetTrajectoryStates().size()";
    return;
  }
  const std::string error =
      map_builder_->SubmapToProto(submap_id, &response_proto);
  if (!error.empty()) {
    LOG(INFO) << error;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    response->status.message = error;
    return;
  }

  response->submap_version = response_proto.submap_version();
  for (const auto& texture_proto : response_proto.textures()) {
    response->textures.emplace_back();
    auto& texture = response->textures.back();
    texture.cells.insert(texture.cells.begin(), texture_proto.cells().begin(),
                         texture_proto.cells().end());
    texture.width = texture_proto.width();
    texture.height = texture_proto.height();
    texture.resolution = texture_proto.resolution();
    texture.slice_pose = ToGeometryMsgPose(
        cartographer::transform::ToRigid3(texture_proto.slice_pose()));
  }
  response->status.message = "Success.";
  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
}

std::map<int, ::cartographer::mapping::PoseGraphInterface::TrajectoryState>
MapBuilderBridge::GetTrajectoryStates() {
  auto trajectory_states = map_builder_->pose_graph()->GetTrajectoryStates();
  // Add active trajectories that are not yet in the pose graph, but are e.g.
  // waiting for input sensor data and thus already have a sensor bridge.
  for (const auto& sensor_bridge : sensor_bridges_) {
    trajectory_states.insert(std::make_pair(
        sensor_bridge.first,
        ::cartographer::mapping::PoseGraphInterface::TrajectoryState::ACTIVE));
  }
  return trajectory_states;
}

cartographer_ros_msgs::msg::SubmapList MapBuilderBridge::GetSubmapList() {
  cartographer_ros_msgs::msg::SubmapList submap_list;
  submap_list.header.stamp = node_ptr_->now();  //::ros::Time::now();
  submap_list.header.frame_id = node_options_.map_frame;
  for (const auto& submap_id_pose :
       map_builder_->pose_graph()->GetAllSubmapPoses()) {
    cartographer_ros_msgs::msg::SubmapEntry submap_entry;
    submap_entry.is_frozen = map_builder_->pose_graph()->IsTrajectoryFrozen(
        submap_id_pose.id.trajectory_id);
    submap_entry.trajectory_id = submap_id_pose.id.trajectory_id;
    submap_entry.submap_index = submap_id_pose.id.submap_index;
    submap_entry.submap_version = submap_id_pose.data.version;
    submap_entry.pose = ToGeometryMsgPose(submap_id_pose.data.pose);
    submap_list.submap.push_back(submap_entry);
  }
  return submap_list;
}

std::unordered_map<int, MapBuilderBridge::LocalTrajectoryData>
MapBuilderBridge::GetLocalTrajectoryData() {
  std::unordered_map<int, LocalTrajectoryData> local_trajectory_data;
  for (const auto& entry : sensor_bridges_) {
    const int trajectory_id = entry.first;
    const SensorBridge& sensor_bridge = *entry.second;

    std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data;
    {
      absl::MutexLock lock(&mutex_);
      if (local_slam_data_.count(trajectory_id) == 0) {
        continue;
      }
      local_slam_data = local_slam_data_.at(trajectory_id);
    }

    // Make sure there is a trajectory with 'trajectory_id'.
    CHECK_EQ(trajectory_options_.count(trajectory_id), 1);
    local_trajectory_data[trajectory_id] = {
        local_slam_data,
        map_builder_->pose_graph()->GetLocalToGlobalTransform(trajectory_id),
        sensor_bridge.tf_bridge().LookupToTracking(
            local_slam_data->time,
            // FromRos(ros::Time::now()),
            trajectory_options_[trajectory_id].published_frame),
        trajectory_options_[trajectory_id]};
  }
  return local_trajectory_data;
}

void MapBuilderBridge::HandleTrajectoryQuery(
    const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response) {
  // This query is safe if the trajectory doesn't exist (returns 0 poses).
  // However, we can filter unwanted states at the higher level in the node.
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  for (const auto& node_id_data :
       node_poses.trajectory(request->trajectory_id)) {
    if (!node_id_data.data.constant_pose_data.has_value()) {
      continue;
    }
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = node_options_.map_frame;
    pose_stamped.header.stamp =
        ToRos(node_id_data.data.constant_pose_data.value().time);
    pose_stamped.pose = ToGeometryMsgPose(node_id_data.data.global_pose);
    response->trajectory.push_back(pose_stamped);
  }
  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  response->status.message = absl::StrCat(
      "Retrieved ", response->trajectory.size(),
      " trajectory nodes from trajectory ", request->trajectory_id, ".");
}

visualization_msgs::msg::MarkerArray MapBuilderBridge::GetTrajectoryNodeList() {
  visualization_msgs::msg::MarkerArray trajectory_node_list;
  const auto node_poses = map_builder_->pose_graph()->GetTrajectoryNodePoses();
  // Find the last node indices for each trajectory that have either
  // inter-submap or inter-trajectory constraints.
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_submap_constrained_node;
  std::map<int, int /* node_index */>
      trajectory_to_last_inter_trajectory_constrained_node;
  for (const int trajectory_id : node_poses.trajectory_ids()) {
    trajectory_to_last_inter_submap_constrained_node[trajectory_id] = 0;
    trajectory_to_last_inter_trajectory_constrained_node[trajectory_id] = 0;
  }
  const auto constraints = map_builder_->pose_graph()->constraints();
  for (const auto& constraint : constraints) {
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTER_SUBMAP) {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        trajectory_to_last_inter_submap_constrained_node[constraint.node_id
                                                             .trajectory_id] =
            std::max(trajectory_to_last_inter_submap_constrained_node.at(
                         constraint.node_id.trajectory_id),
                     constraint.node_id.node_index);
      } else {
        trajectory_to_last_inter_trajectory_constrained_node
            [constraint.node_id.trajectory_id] =
                std::max(trajectory_to_last_inter_submap_constrained_node.at(
                             constraint.node_id.trajectory_id),
                         constraint.node_id.node_index);
      }
    }
  }

  for (const int trajectory_id : node_poses.trajectory_ids()) {
    visualization_msgs::msg::Marker marker =
        CreateTrajectoryMarker(trajectory_id, node_options_.map_frame, node_ptr_->now());
    int last_inter_submap_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_submap_constrained_node.at(trajectory_id));
    int last_inter_trajectory_constrained_node = std::max(
        node_poses.trajectory(trajectory_id).begin()->id.node_index,
        trajectory_to_last_inter_trajectory_constrained_node.at(trajectory_id));
    last_inter_submap_constrained_node =
        std::max(last_inter_submap_constrained_node,
                 last_inter_trajectory_constrained_node);

    if (map_builder_->pose_graph()->IsTrajectoryFrozen(trajectory_id)) {
      last_inter_submap_constrained_node =
          (--node_poses.trajectory(trajectory_id).end())->id.node_index;
      last_inter_trajectory_constrained_node =
          last_inter_submap_constrained_node;
    }

    marker.color.a = 1.0;
    for (const auto& node_id_data : node_poses.trajectory(trajectory_id)) {
      if (!node_id_data.data.constant_pose_data.has_value()) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        continue;
      }
      const ::geometry_msgs::msg::Point node_point =
          ToGeometryMsgPoint(node_id_data.data.global_pose.translation());
      marker.points.push_back(node_point);

      if (node_id_data.id.node_index ==
          last_inter_trajectory_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.5;
      }
      if (node_id_data.id.node_index == last_inter_submap_constrained_node) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        marker.points.push_back(node_point);
        marker.color.a = 0.25;
      }
      // Work around the 16384 point limit in RViz by splitting the
      // trajectory into multiple markers.
      if (marker.points.size() == 16384) {
        PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
        // Push back the last point, so the two markers appear connected.
        marker.points.push_back(node_point);
      }
    }
    PushAndResetLineMarker(&marker, &trajectory_node_list.markers);
    size_t current_last_marker_id = static_cast<size_t>(marker.id - 1);
    if (trajectory_to_highest_marker_id_.count(trajectory_id) == 0) {
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    } else {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      while (static_cast<size_t>(marker.id) <=
             trajectory_to_highest_marker_id_[trajectory_id]) {
        trajectory_node_list.markers.push_back(marker);
        ++marker.id;
      }
      trajectory_to_highest_marker_id_[trajectory_id] = current_last_marker_id;
    }
  }
  return trajectory_node_list;
}

visualization_msgs::msg::MarkerArray MapBuilderBridge::GetLandmarkPosesList() {
  visualization_msgs::msg::MarkerArray landmark_poses_list;
  const std::map<std::string, Rigid3d> landmark_poses =
      map_builder_->pose_graph()->GetLandmarkPoses();
  for (const auto& id_to_pose : landmark_poses) {
    std::string idString = id_to_pose.first;
    landmark_poses_list.markers.push_back(CreateLandmarkMarker(
        id_to_pose.first,
        id_to_pose.second, node_options_.map_frame,
        node_ptr_->now()));
  }
  return landmark_poses_list;
}

visualization_msgs::msg::MarkerArray MapBuilderBridge::GetConstraintList() {
  visualization_msgs::msg::MarkerArray constraint_list;
  int marker_id = 0;
  visualization_msgs::msg::Marker constraint_intra_marker;
  constraint_intra_marker.id = marker_id++;
  constraint_intra_marker.ns = "Intra constraints";
  constraint_intra_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  constraint_intra_marker.header.stamp = node_ptr_->now();  // ros::Time::now();
  constraint_intra_marker.header.frame_id = node_options_.map_frame;
  constraint_intra_marker.scale.x = kConstraintMarkerScale;
  constraint_intra_marker.pose.orientation.w = 1.0;

  visualization_msgs::msg::Marker residual_intra_marker = constraint_intra_marker;
  residual_intra_marker.id = marker_id++;
  residual_intra_marker.ns = "Intra residuals";
  // This and other markers which are less numerous are set to be slightly
  // above the intra constraints marker in order to ensure that they are
  // visible.
  residual_intra_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker constraint_inter_same_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_same_trajectory_marker.id = marker_id++;
  constraint_inter_same_trajectory_marker.ns =
      "Inter constraints, same trajectory";
  constraint_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker residual_inter_same_trajectory_marker =
      constraint_intra_marker;
  residual_inter_same_trajectory_marker.id = marker_id++;
  residual_inter_same_trajectory_marker.ns = "Inter residuals, same trajectory";
  residual_inter_same_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker constraint_inter_diff_trajectory_marker =
      constraint_intra_marker;
  constraint_inter_diff_trajectory_marker.id = marker_id++;
  constraint_inter_diff_trajectory_marker.ns =
      "Inter constraints, different trajectories";
  constraint_inter_diff_trajectory_marker.pose.position.z = 0.1;

  visualization_msgs::msg::Marker residual_inter_diff_trajectory_marker =
      constraint_intra_marker;
  residual_inter_diff_trajectory_marker.id = marker_id++;
  residual_inter_diff_trajectory_marker.ns =
      "Inter residuals, different trajectories";
  residual_inter_diff_trajectory_marker.pose.position.z = 0.1;

  const auto trajectory_node_poses =
      map_builder_->pose_graph()->GetTrajectoryNodePoses();
  const auto submap_poses = map_builder_->pose_graph()->GetAllSubmapPoses();
  const auto constraints = map_builder_->pose_graph()->constraints();

  for (const auto& constraint : constraints) {
    visualization_msgs::msg::Marker *constraint_marker, *residual_marker;
    std_msgs::msg::ColorRGBA color_constraint, color_residual;
    if (constraint.tag ==
        cartographer::mapping::PoseGraphInterface::Constraint::INTRA_SUBMAP) {
      constraint_marker = &constraint_intra_marker;
      residual_marker = &residual_intra_marker;
      // Color mapping for submaps of various trajectories - add trajectory id
      // to ensure different starting colors. Also add a fixed offset of 25
      // to avoid having identical colors as trajectories.
      color_constraint = ToMessage(
          cartographer::io::GetColor(constraint.submap_id.submap_index +
                                     constraint.submap_id.trajectory_id + 25));
      color_residual.a = 1.0;
      color_residual.r = 1.0;
    } else {
      if (constraint.node_id.trajectory_id ==
          constraint.submap_id.trajectory_id) {
        constraint_marker = &constraint_inter_same_trajectory_marker;
        residual_marker = &residual_inter_same_trajectory_marker;
        // Bright yellow
        color_constraint.a = 1.0;
        color_constraint.r = color_constraint.g = 1.0;
      } else {
        constraint_marker = &constraint_inter_diff_trajectory_marker;
        residual_marker = &residual_inter_diff_trajectory_marker;
        // Bright orange
        color_constraint.a = 1.0;
        color_constraint.r = 1.0;
        color_constraint.g = 165. / 255.;
      }
      // Bright cyan
      color_residual.a = 1.0;
      color_residual.b = color_residual.g = 1.0;
    }

    for (int i = 0; i < 2; ++i) {
      constraint_marker->colors.push_back(color_constraint);
      residual_marker->colors.push_back(color_residual);
    }

    const auto submap_it = submap_poses.find(constraint.submap_id);
    if (submap_it == submap_poses.end()) {
      continue;
    }
    const auto& submap_pose = submap_it->data.pose;
    const auto node_it = trajectory_node_poses.find(constraint.node_id);
    if (node_it == trajectory_node_poses.end()) {
      continue;
    }
    const auto& trajectory_node_pose = node_it->data.global_pose;
    const Rigid3d constraint_pose = submap_pose * constraint.pose.zbar_ij;

    constraint_marker->points.push_back(
        ToGeometryMsgPoint(submap_pose.translation()));
    constraint_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));

    residual_marker->points.push_back(
        ToGeometryMsgPoint(constraint_pose.translation()));
    residual_marker->points.push_back(
        ToGeometryMsgPoint(trajectory_node_pose.translation()));
  }

  constraint_list.markers.push_back(constraint_intra_marker);
  constraint_list.markers.push_back(residual_intra_marker);
  constraint_list.markers.push_back(constraint_inter_same_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_same_trajectory_marker);
  constraint_list.markers.push_back(constraint_inter_diff_trajectory_marker);
  constraint_list.markers.push_back(residual_inter_diff_trajectory_marker);
  return constraint_list;
}

SensorBridge* MapBuilderBridge::sensor_bridge(const int trajectory_id) {
  return sensor_bridges_.at(trajectory_id).get();
}

void MapBuilderBridge::OnLocalSlamResult(
    const int trajectory_id, const ::cartographer::common::Time time,
    const Rigid3d local_pose,
    ::cartographer::sensor::RangeData range_data_in_local) {
  std::shared_ptr<const LocalTrajectoryData::LocalSlamData> local_slam_data =
      std::make_shared<LocalTrajectoryData::LocalSlamData>(
          LocalTrajectoryData::LocalSlamData{time, local_pose,
                                             std::move(range_data_in_local)});
  absl::MutexLock lock(&mutex_);
  local_slam_data_[trajectory_id] = std::move(local_slam_data);
}

bool MapBuilderBridge::CheckSwitchMapFinish()
{
  return map_builder_->pose_graph()->GetTrajectoryNodes().size()>3;
}


}  // namespace cartographer_ros
