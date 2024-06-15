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

#include "cartographer/mapping/internal/constraints/constraint_builder_2d.h"

#include <cmath>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "Eigen/Eigenvalues"
#include "absl/memory/memory.h"
#include "cartographer/common/math.h"
#include "cartographer/common/thread_pool.h"
#include "cartographer/mapping/proto/scan_matching/ceres_scan_matcher_options_2d.pb.h"
#include "cartographer/mapping/proto/scan_matching/fast_correlative_scan_matcher_options_2d.pb.h"
#include "cartographer/metrics/counter.h"
#include "cartographer/metrics/gauge.h"
#include "cartographer/metrics/histogram.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace constraints {

static auto* kConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kConstraintsFoundMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsSearchedMetric = metrics::Counter::Null();
static auto* kGlobalConstraintsFoundMetric = metrics::Counter::Null();
static auto* kQueueLengthMetric = metrics::Gauge::Null();
static auto* kConstraintScoresMetric = metrics::Histogram::Null();
static auto* kGlobalConstraintScoresMetric = metrics::Histogram::Null();
static auto* kNumSubmapScanMatchersMetric = metrics::Gauge::Null();

transform::Rigid2d ComputeSubmapPose(const Submap2D& submap) {
  return transform::Project2D(submap.local_pose());
}

ConstraintBuilder2D::ConstraintBuilder2D(
    const constraints::proto::ConstraintBuilderOptions& options,
    common::ThreadPoolInterface* const thread_pool, std::shared_ptr<cartographer::GlobalMessage>& globalMessagePtr)
    : options_(options),
      thread_pool_(thread_pool),
      finish_node_task_(absl::make_unique<common::Task>()),
      when_done_task_(absl::make_unique<common::Task>()),
      ceres_scan_matcher_(options.ceres_scan_matcher_options()), maybeLongCorridor_(false), initFinish_(false),
      linearCumulateSinceLost_(0.0), angularCumulateSinceLost_(0.0), 
      globalPose2dLastLost_(transform::Rigid2d::Identity()), poseLostInit_(false), scorePoseFromMapPtr_(new double),
      globalMessagePtr_(globalMessagePtr) {
        LOG(INFO)<<"ConstraintBuilder2D !!!";
      }

ConstraintBuilder2D::~ConstraintBuilder2D() {
  absl::MutexLock locker(&mutex_);
  CHECK_EQ(finish_node_task_->GetState(), common::Task::NEW);
  CHECK_EQ(when_done_task_->GetState(), common::Task::NEW);
  CHECK_EQ(constraints_.size(), 0) << "WhenDone() was not called";
  CHECK_EQ(num_started_nodes_, num_finished_nodes_);
  CHECK(when_done_ == nullptr);
}

void ConstraintBuilder2D::MaybeAddConstraint(const Submap2D* src_submap, const SubmapId& tgt_submap_id, const Submap2D* tgt_submap,
                          const NodeId& node_id,
                          const TrajectoryNode::Data* const constant_data,
                          const transform::Rigid2d& initial_relative_pose,
                          const transform::Rigid2d& global_pose_2d,
                          const bool& maybeLongCorridor){
  maybeLongCorridor_ = maybeLongCorridor;
  if (node_id.trajectory_id == tgt_submap_id.trajectory_id && initial_relative_pose.translation().norm() >
      options_.max_constraint_distance()) {
    return;
  }
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(tgt_submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }

  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher = DispatchScanMatcherConstruction(tgt_submap_id, tgt_submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(tgt_submap_id, tgt_submap, node_id, CONSTRAINT_LOCAL, /* match_full_submap */
                      constant_data, initial_relative_pose, global_pose_2d, *scan_matcher,
                      constraint, src_submap);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::MaybeAddInitConstraint(const Submap2D* src_submap, const SubmapId& tgt_submap_id, const Submap2D* const tgt_submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const transform::Rigid2d& global_pose_2d) {
  // if (initial_relative_pose.translation().norm() >
  //     options_.max_constraint_distance()) {
  //   return;
  // }
  if (!per_submap_sampler_
           .emplace(std::piecewise_construct, std::forward_as_tuple(tgt_submap_id),
                    std::forward_as_tuple(options_.sampling_ratio()))
           .first->second.Pulse()) {
    return;
  }
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddInitConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(tgt_submap_id, tgt_submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(tgt_submap_id, tgt_submap, node_id, CONSTRAINT_INIT, /* match_map */
                      constant_data, initial_relative_pose, global_pose_2d, *scan_matcher,
                      constraint, src_submap);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const transform::Rigid2d& global_pose_2d) {
  // if (!per_submap_sampler_
  //          .emplace(std::piecewise_construct, std::forward_as_tuple(submap_id),
  //                   std::forward_as_tuple(options_.sampling_ratio()))
  //          .first->second.Pulse()) {
  //   return;
  // }

  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.clear();
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, CONSTRAINT_FULL, /* match_full_map */
                      constant_data, initial_relative_pose, global_pose_2d, *scan_matcher,
                      constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::MaybeAddGlobalConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, const TrajectoryNode::Data* const constant_data) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "MaybeAddGlobalConstraint was called while WhenDone was scheduled.";
  }
  constraints_.emplace_back();
  kQueueLengthMetric->Set(constraints_.size());
  auto* const constraint = &constraints_.back();
  const auto* scan_matcher =
      DispatchScanMatcherConstruction(submap_id, submap->grid());
  auto constraint_task = absl::make_unique<common::Task>();
  constraint_task->SetWorkItem([=]() LOCKS_EXCLUDED(mutex_) {
    ComputeConstraint(submap_id, submap, node_id, CONSTRAINT_FULL, /* match_full_submap */
                      constant_data, transform::Rigid2d::Identity(), transform::Rigid2d::Identity(),
                      *scan_matcher, constraint);
  });
  constraint_task->AddDependency(scan_matcher->creation_task_handle);
  auto constraint_task_handle =
      thread_pool_->Schedule(std::move(constraint_task));
  finish_node_task_->AddDependency(constraint_task_handle);
}

void ConstraintBuilder2D::NotifyEndOfNode() {
  absl::MutexLock locker(&mutex_);
  CHECK(finish_node_task_ != nullptr);
  finish_node_task_->SetWorkItem([this] {
    absl::MutexLock locker(&mutex_);
    ++num_finished_nodes_;
  });
  auto finish_node_task_handle =
      thread_pool_->Schedule(std::move(finish_node_task_));
  finish_node_task_ = absl::make_unique<common::Task>();
  when_done_task_->AddDependency(finish_node_task_handle);
  ++num_started_nodes_;
}

void ConstraintBuilder2D::WhenDone(
    const std::function<void(const ConstraintBuilder2D::Result&)>& callback) {
  absl::MutexLock locker(&mutex_);
  CHECK(when_done_ == nullptr);
  // TODO(gaschler): Consider using just std::function, it can also be empty.
  when_done_ = absl::make_unique<std::function<void(const Result&)>>(callback);
  CHECK(when_done_task_ != nullptr);
  when_done_task_->SetWorkItem([this] { RunWhenDoneCallback(); });
  thread_pool_->Schedule(std::move(when_done_task_));
  when_done_task_ = absl::make_unique<common::Task>();
}

const ConstraintBuilder2D::SubmapScanMatcher*
ConstraintBuilder2D::DispatchScanMatcherConstruction(const SubmapId& submap_id,
                                                     const Grid2D* const grid) {
  CHECK(grid);
  if (submap_scan_matchers_.count(submap_id) != 0) {
    return &submap_scan_matchers_.at(submap_id);
  }
  auto& submap_scan_matcher = submap_scan_matchers_[submap_id];
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
  submap_scan_matcher.grid = grid;
  auto& scan_matcher_options = options_.fast_correlative_scan_matcher_options();
  auto scan_matcher_task = absl::make_unique<common::Task>();
  scan_matcher_task->SetWorkItem(
      [&submap_scan_matcher, &scan_matcher_options]() {
        submap_scan_matcher.fast_correlative_scan_matcher =
            absl::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                *submap_scan_matcher.grid, scan_matcher_options);
      });
  submap_scan_matcher.creation_task_handle =
      thread_pool_->Schedule(std::move(scan_matcher_task));
  return &submap_scan_matchers_.at(submap_id);
}

void ConstraintBuilder2D::ComputeConstraint(
    const SubmapId& submap_id, const Submap2D* const submap,
    const NodeId& node_id, ModeConstraint mode_constraint,
    const TrajectoryNode::Data* const constant_data,
    const transform::Rigid2d& initial_relative_pose,
    const transform::Rigid2d& global_pose_2d,
    const SubmapScanMatcher& submap_scan_matcher,
    std::unique_ptr<ConstraintBuilder2D::Constraint>* constraint,
    const Submap2D* src_submap) {
  CHECK(submap_scan_matcher.fast_correlative_scan_matcher);
  sensor::PointCloud src_submap_pointcloud; //local系
  
  if(src_submap != nullptr && submap_id.trajectory_id == 0 && node_id.trajectory_id != 0) //跟地图匹配才去计算子图的点云
  {
    sensor::PointCloud point_cloud( src_submap->GetRangeData(globalMessagePtr_->getSubmapFreeThreshold()) );
    src_submap_pointcloud = sensor::TransformPointCloud(point_cloud, constant_data->local_pose.inverse().cast<float>());//转换到local原点
    // src_submap_pointcloud.push_back(sensor::PointCloud::PointType{Eigen::Vector3f(0.0,0.0,0.0)});
    globalMessagePtr_->setTimedSubmapPointcloud(constant_data->time, std::make_shared<sensor::PointCloud>(src_submap_pointcloud.points()));
    LOG(INFO) << "free_cost < "<< globalMessagePtr_->getSubmapFreeThreshold() << ",sub map points=" << src_submap_pointcloud.size() << ",submap range num:" \
        << src_submap->num_range_data() << ",node points=" << constant_data->filtered_gravity_aligned_point_cloud.size() <<" -----------------0";
  }
  if ((2 * src_submap_pointcloud.size()) < constant_data->filtered_gravity_aligned_point_cloud.size())
  {
    src_submap_pointcloud = constant_data->filtered_gravity_aligned_point_cloud;
  }
  const transform::Rigid2d initial_pose =
      ComputeSubmapPose(*submap) * initial_relative_pose;
  
  const transform::Rigid2d local_to_global = global_pose_2d * initial_pose.inverse();
  // The 'constraint_transform' (submap i <- node j) is computed from:
  // - a 'filtered_gravity_aligned_point_cloud' in node j,
  // - the initial guess 'initial_pose' for (map <- node j),
  // - the result 'pose_estimate' of Match() (map <- node j).
  // - the ComputeSubmapPose() (map <- submap i)
  float score = 0.;
  int threshold_valid_lidar_points = 100;
  bool flag_less_valid_lidar_points = false;
  double translation_weight_with_map = 2e6;
  double rotation_weight_with_map = 4e6;
  float min_score_with_map = options_.min_score_with_map();
  
  // float min_score_init = options_.min_score_init();
  bool flagMatchMap = true;

  if(constant_data->filtered_gravity_aligned_point_cloud.size() < threshold_valid_lidar_points){
    LOG(INFO) << "valid lidar points num: " << constant_data->filtered_gravity_aligned_point_cloud.size();
    flag_less_valid_lidar_points = true;
  }

  if(submap_id.trajectory_id == 0 && node_id.trajectory_id != 0){
    if(globalMessagePtr_ != nullptr){
      flagMatchMap = globalMessagePtr_->getFlagMatchMap();
    }
    else{
      LOG(ERROR) << "constraint_builder_2d globalMessagePtr_ is nullptr";
    }
  }
  transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();
  double scoreVisualSlam = 0.0;
  double scoreTagPose = 0.0;
  PoseWithScore pose_score_visual_slam = { transform::Rigid2d::Identity(), 0.0};
  PoseWithScore pose_score_tag_pose = { transform::Rigid2d::Identity(), 0.0};
  bool flagVisualSlamIsGood = false;
  bool flagTagPoseIsGood = false;
  // Compute 'pose_estimate' in three stages:
  // 1. Fast estimate using the fast correlative scan matcher.
  // 2. Prune if the score is too low.
  // 3. Refine.
  // mode_constraint = CONSTRAINT_LOCAL;
  float scoreFromMap = 0.0f;
  auto pose_estimate_temp = pose_estimate;
  if(submap_id.trajectory_id == 0 && node_id.trajectory_id != 0){
    submap_scan_matcher.fast_correlative_scan_matcher->Match(
                  initial_pose, 
                  src_submap_pointcloud,
                  -100.0,
                  -100.0, //10.0 / 180.0 * 3.14159
                  0.1f, &scoreFromMap, &pose_estimate_temp);
    *scorePoseFromMapPtr_ = scoreFromMap;
    // LOG(INFO) << "scoreFromMap: " << scoreFromMap;
  }
  //tagPose
  if(globalMessagePtr_ != nullptr && globalMessagePtr_->getFlagUseTagPose() && submap_id.trajectory_id == 0 && node_id.trajectory_id != 0){
    pose_score_tag_pose = globalMessagePtr_->getPoseFromTag();
    pose_score_tag_pose.pose = ComputeSubmapPose(*submap) * pose_score_tag_pose.pose;
    scoreTagPose = pose_score_tag_pose.score;
    score = scoreTagPose;
    if(scoreTagPose > 0.8){
      flagTagPoseIsGood = true;
    }
    else{
      flagTagPoseIsGood = false;
    }
  }
  //vslamPose
  else if(globalMessagePtr_ != nullptr && globalMessagePtr_->getFlagUseVisualSlamPose() && submap_id.trajectory_id == 0 && node_id.trajectory_id != 0){
    pose_score_visual_slam = globalMessagePtr_->getPoseFromVisualSlam();
    pose_score_visual_slam.pose = ComputeSubmapPose(*submap) * pose_score_visual_slam.pose;
    scoreVisualSlam = pose_score_visual_slam.score;
    score = scoreVisualSlam;
    if(scoreVisualSlam > 0.8){
      auto pose_estimate_visual = pose_estimate;
      float score_match_map_from_visual = 0.0f;
      submap_scan_matcher.fast_correlative_scan_matcher->Match(
                    pose_score_visual_slam.pose, constant_data->filtered_gravity_aligned_point_cloud,
                    0.1,
                    0.17, //10.0 / 180.0 * 3.14159
                    0.2f, &score_match_map_from_visual, &pose_estimate_visual);
      if(flag_less_valid_lidar_points
         || (score_match_map_from_visual > scoreFromMap - 0.05f && score_match_map_from_visual > 0.3f)
      ){
        // LOG(INFO) << "flagVisualSlamIsGood = true";
        flagVisualSlamIsGood = true;
        pose_score_visual_slam.pose = pose_estimate_visual;
        translation_weight_with_map = 2e6;
        rotation_weight_with_map = 6e6;
      }
      else{
        flagVisualSlamIsGood = false;
        LOG(INFO) << "score_match_map_from_visual: " << score_match_map_from_visual << " scoreFromMap: " << scoreFromMap;
        LOG(WARNING) << "flagVisualSlamIsGood = false";
      }
    }
    else{
      // LOG(INFO) << "pose_score_visual_slam.score:  " << pose_score_visual_slam.score;
      flagVisualSlamIsGood = false;
    }
  }

  if(flagTagPoseIsGood){
    LOG(INFO) << "pose_score_tag.score:  " << pose_score_visual_slam.score;
  }
  else if(flagVisualSlamIsGood){
    // LOG(INFO) << "pose_score_visual_slam.score:  " << pose_score_visual_slam.score;
  }
  else if(flag_less_valid_lidar_points &&  (src_submap_pointcloud.size() < constant_data->filtered_gravity_aligned_point_cloud.size())){
    // LOG(INFO) << "node.points size: " << constant_data->filtered_gravity_aligned_point_cloud.size();
    return;
  }
  else if(mode_constraint == CONSTRAINT_FULL) {
    kGlobalConstraintsSearchedMetric->Increment();
    bool successMatch = false;
    for(int i = 0; i <= 0; i++){
      successMatch = submap_scan_matcher.fast_correlative_scan_matcher->Match(
          initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
          std::min((3.0 + 2.0 * i) * 10 , 10.0 * 10),
          (20.0 + (10 * i) * 3.14 / 180) * 200,
          options_.global_localization_min_score() - 0.05 * i, &score, &pose_estimate);
      LOG(INFO) << " try global localization " << i << " score: " << score << " min_score: " << options_.global_localization_min_score() - 0.05 * i;
      if(successMatch){
        LOG(WARNING) << "lidar slam relocalization success!" << " score: " << score;
        translation_weight_with_map = 1e8;
        rotation_weight_with_map = 1e8;
        break;
      }
    }
    if(globalMessagePtr_ != nullptr && globalMessagePtr_->getFlagUseTagPose()){
      PoseWithScore pose_tag_global = globalMessagePtr_->getPoseFromTag();
      pose_estimate = local_to_global.inverse() * pose_tag_global.pose;
      score = 0.9;
      successMatch = true;
      LOG(INFO) << "try global localization from tag";
      LOG(INFO) << "pose_tag_global: \n" << pose_tag_global.pose; 
      LOG(INFO) << "local_to_global: \n" << local_to_global; 
      LOG(INFO) << "pose_estimate: \n" << pose_estimate; 

    }
    else if(globalMessagePtr_ != nullptr && globalMessagePtr_->getFlagUseVisualSlamPose()){
      PoseWithScore pose_visual_global = globalMessagePtr_->getPoseFromVisualSlam();
      pose_estimate = local_to_global.inverse() * pose_visual_global.pose;
      LOG(INFO) << "try global localization from visual slam";
      LOG(INFO) << "pose_visual_global: \n" << pose_visual_global.pose; 
      LOG(INFO) << "local_to_global: \n" << local_to_global; 
      LOG(INFO) << "pose_estimate: \n" << pose_estimate; 
      if(pose_visual_global.score > 0.2){
        score = pose_visual_global.score;
        successMatch = true;
        LOG(WARNING) << "visual slam relocalization success!" << " score: " << pose_score_visual_slam.score;
      }

    }
    *scorePoseFromMapPtr_ = score;
    if (successMatch) {
      CHECK_GT(score, options_.global_localization_min_score());
      CHECK_GE(node_id.trajectory_id, 0);   
      CHECK_GE(submap_id.trajectory_id, 0);
      LOG(INFO)<< "GlobalLocalization success!";
      translation_weight_with_map = 5e7;
      rotation_weight_with_map = 5e9;
      kGlobalConstraintsFoundMetric->Increment();
      kGlobalConstraintScoresMetric->Observe(score);
    } else {
      LOG(WARNING)<< "GlobalLocalization false!";
      return;
    }
  } 
  else if (mode_constraint == CONSTRAINT_LOCAL){    
    // LOG(INFO) << "mode_constraint = CONSTRAINT_LOCAL";      
    kConstraintsSearchedMetric->Increment();
    if(submap_id.trajectory_id == 0 && node_id.trajectory_id != 0){
      if(linearCumulateSinceLost_ < 9.0f){
        min_score_with_map = min_score_with_map + (0.18f - linearCumulateSinceLost_ * 0.02f);
        // LOG(INFO) << "linearCumulateSinceLost_ :" << linearCumulateSinceLost_;      
        // min_score_with_map = std::min(min_score_with_map + (0.2f - linearCumulateSinceLost_ * 0.02f), 0.78);
      }
      else if(linearCumulateSinceLost_ > 15.0f){
        linearCumulateSinceLost_ = 15.0f;
      }
      if(flagMatchMap == false){
        min_score_with_map = options_.min_score_with_map_no_match();
        // LOG(INFO) << "constraint_builder: min_score_with_map:" << min_score_with_map;
      }
      // LOG(INFO) << "constraint_builder: min_score_with_map:" << min_score_with_map << " linearCumulateSinceLost_: " << linearCumulateSinceLost_;
      // float scoreFromMap = 0.0f;
      // auto pose_estimate_temp = pose_estimate;
      // submap_scan_matcher.fast_correlative_scan_matcher->Match(
      //               initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
      //               -100.0,
      //               -100.0,
      //               0.1f, &scoreFromMap, &pose_estimate_temp);
      // *scorePoseFromMapPtr_ = scoreFromMap;
      // LOG(INFO) << "pose score from map: " << *scorePoseFromMapPtr_;
      bool successMatch = false;
      successMatch = submap_scan_matcher.fast_correlative_scan_matcher->Match(
              initial_pose, 
              src_submap_pointcloud, 
              linearCumulateSinceLost_,
              angularCumulateSinceLost_,
              min_score_with_map, &score, &pose_estimate);
      // LOG(INFO) << "score: " << score << " min_score_with_map: " << min_score_with_map;
      if (successMatch && (pose_estimate.translation() - initial_pose.translation()).norm() < 5.0f && maybeLongCorridor_ == false) {
        // We've reported a successful local match.
        CHECK_GT(score, min_score_with_map);
        if(linearCumulateSinceLost_ < 5.0f){
          linearCumulateSinceLost_ = std::max((double) 0.0f, linearCumulateSinceLost_ - 0.2f);
        }
        else{
          linearCumulateSinceLost_ = std::max((double) 0.0f, linearCumulateSinceLost_ - 1.0f);
        }
        angularCumulateSinceLost_ = std::max((double) 0.0f, angularCumulateSinceLost_ - 0.1f);
        poseLostInit_ = false;
        kConstraintsFoundMetric->Increment();
        kConstraintScoresMetric->Observe(score);
      } 
      else {
        if(!poseLostInit_ && linearCumulateSinceLost_ < 0.1f){
          globalPose2dLastLost_ = global_pose_2d;
          poseLostInit_ = true;
        }
        else{
          auto translation_diff = std::min((global_pose_2d.translation() - globalPose2dLastLost_.translation()).norm(), 0.2);
          auto angle_diff = fabs(global_pose_2d.normalized_angle() - globalPose2dLastLost_.normalized_angle());
          // LOG(INFO)<< "translation_diff: "<< translation_diff << "  angle_diff: " << angle_diff;
          linearCumulateSinceLost_ += translation_diff;
          angularCumulateSinceLost_ += angle_diff;
          angularCumulateSinceLost_ = std::min(angularCumulateSinceLost_, 87.);
          if(globalMessagePtr_ != nullptr && !globalMessagePtr_->getFlagIsLocalization()){
            linearCumulateSinceLost_ = std::min(linearCumulateSinceLost_, 5.);
            angularCumulateSinceLost_ = std::min(angularCumulateSinceLost_, 40.);          
          }
          globalPose2dLastLost_ = global_pose_2d;
        }
        // LOG(WARNING) << " diff pose norm: " << (pose_estimate.translation() - initial_pose.translation()).norm() << " ifLong: " << maybeLongCorridor_;
        // LOG(INFO) << "pose_estimate.translation(): \n" << pose_estimate.translation();
        // LOG(INFO) << "initial_pose.translation(): \n" << initial_pose.translation();
        return;
      }
    }
    else{
      if(globalMessagePtr_ != nullptr && !globalMessagePtr_->getFlagHasSufficientFeatures()){
        LOG(WARNING) << "local laser scan not has sufficient features, do not add constraint";
        return;
      }
      if (submap_scan_matcher.fast_correlative_scan_matcher->Match(
              initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
              options_.min_score(), &score, &pose_estimate)) {
        // We've reported a successful local match.
        CHECK_GT(score, options_.min_score());
        kConstraintsFoundMetric->Increment();
        kConstraintScoresMetric->Observe(score);
        // LOG(WARNING) << "node_id.node_index: " << node_id.node_index << " submap_id " << submap_id.submap_index <<  " Constraint score: " << score;
      } else {
        // LOG(INFO) << "node_id.node_index: " << node_id.node_index << " submap_id " << submap_id.submap_index <<  " Constraint score: " << score;
        return;
      }      
    }
  }
  else{
    if(globalMessagePtr_ != nullptr && !globalMessagePtr_->getFlagHasSufficientFeatures()){
        LOG(WARNING) << "init laser scan not has sufficient features, do not add constraint";
        return;
    }
    // LOG(INFO) << "mode_constraint = CONSTRAINT_INIT, options_.min_score_init: " << options_.min_score_init();      
    kConstraintsSearchedMetric->Increment();
    bool multi_init_poses = false;
    std::vector<transform::Rigid2d> init_poses = globalMessagePtr_->getMultiInitPoses();
    if (init_poses.size() > 1) {
      LOG(INFO) << "build init constraint with multi initial poses.";
      multi_init_poses = true;
    }

    bool successMatchInitMap = false;
    if (multi_init_poses) { // using multi initial poses to initialize
      double max_score = 0.0;
      transform::Rigid2d best_pose = transform::Rigid2d::Identity();
      if(flagMatchMap == true) {
        for (const auto pose : init_poses) {
          bool ret = submap_scan_matcher.fast_correlative_scan_matcher->MatchInitMap(
                  pose, constant_data->filtered_gravity_aligned_point_cloud,
                  options_.min_score_init(), &score, &pose_estimate);
          if (ret) {
            successMatchInitMap = true;
            if (score > max_score) {
              max_score = score;
              best_pose = pose_estimate;
            }
          }
        }
      } else {
        for (const auto pose : init_poses) {
          bool ret = submap_scan_matcher.fast_correlative_scan_matcher->MatchInitMapNoMatch(
                  pose, constant_data->filtered_gravity_aligned_point_cloud,
                  options_.min_score_init_no_match(), &score, &pose_estimate); 
          if (ret) {
            successMatchInitMap = true;
            if (score > max_score) {
              max_score = score;
              best_pose = pose_estimate;
            }
          }
        }
      }
      if (successMatchInitMap) {
        LOG(INFO) << "successfully match the init map with pose: " << best_pose;
        pose_estimate = best_pose;
      }
      LOG(INFO) << "Init constraint *score: " << max_score;
    } else { // normal case
      if(flagMatchMap == true) {
        successMatchInitMap = submap_scan_matcher.fast_correlative_scan_matcher->MatchInitMap(
                initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
                options_.min_score_init(), &score, &pose_estimate);
      } else {
        successMatchInitMap = submap_scan_matcher.fast_correlative_scan_matcher->MatchInitMapNoMatch(
                initial_pose, constant_data->filtered_gravity_aligned_point_cloud,
                options_.min_score_init_no_match(), &score, &pose_estimate);         
      }
      LOG(INFO) << "Init constraint *score: " << score;
    }

    //if(submap_id.trajectory_id == 0){
    //  *scorePoseFromMapPtr_ = score;
    //}    
    if (successMatchInitMap) {
      // We've reported a successful local match.
      initFinish_ = true;
      translation_weight_with_map = 5e7;
      rotation_weight_with_map = 5e7;
      kConstraintsFoundMetric->Increment();
      kConstraintScoresMetric->Observe(score);
    } else {
      LOG(INFO) << "Match init map failed.";
      return;
    }
  }
  {
    absl::MutexLock locker(&mutex_);
    score_histogram_.Add(score);
  }

  if (!initFinish_ && (flagTagPoseIsGood || flagVisualSlamIsGood)) {
    LOG(INFO) << "Tag pose or VSLAM pose is good, Init constrain finished.";
    initFinish_ = true;
  }

  // Use the CSM estimate as both the initial and previous pose. This has the
  // effect that, in the absence of better information, we prefer the original
  // CSM estimate.
  ceres::Solver::Summary unused_summary;
  if(flagTagPoseIsGood  && submap_id.trajectory_id == 0 && node_id.trajectory_id != 0){
    pose_estimate =   pose_score_tag_pose.pose;
  }
  else if(flagVisualSlamIsGood  && submap_id.trajectory_id == 0 && node_id.trajectory_id != 0){
    pose_estimate =   pose_score_visual_slam.pose;
  }
  // else{
    ceres_scan_matcher_.Match(pose_estimate.translation(), pose_estimate,
                              constant_data->filtered_gravity_aligned_point_cloud,
                              *submap_scan_matcher.grid, &pose_estimate,
                              &unused_summary, false);
  // }
  
  const transform::Rigid2d constraint_transform =
      ComputeSubmapPose(*submap).inverse() * pose_estimate;
  const transform::Rigid2d pose_estimate_global = local_to_global * pose_estimate;
  const MapLimits& limits = submap_scan_matcher.grid->limits();
  float correspondence_cost = submap_scan_matcher.grid->GetCorrespondenceCostSelf(Eigen::Array2i((limits.max().y() - pose_estimate_global.translation()[1]) 
                                                                                                        / limits.resolution() - 0.5,
                                                                                        (limits.max().x() - pose_estimate_global.translation()[0]) 
                                                                                                        / limits.resolution() - 0.5));
  auto diff_trans = (global_pose_2d.translation() - pose_estimate.translation()).norm();
  auto trans_window = linearCumulateSinceLost_ * 0.15 + 2.;
  if((correspondence_cost < 0.52 || correspondence_cost > 1.0) && submap_id.trajectory_id == 0){
    LOG(WARNING) << "constraint is invalid";
    LOG(INFO) << "pose_estimate_global.translation : \n" << pose_estimate_global.translation();
    LOG(INFO) << "pose_estimate_global correspondence_cost : \n" << correspondence_cost;
    if (globalMessagePtr_->getInitState() == cartographer::InitState::BUILD_CONSTRAINT) {
      initFinish_ = false;
      globalMessagePtr_->setFlagInitConstraint(false);
      LOG(INFO) << "Check Init Constraint Invalid.";
    }
    return;
  }
  if(maybeLongCorridor_ == false || mode_constraint == CONSTRAINT_FULL || flagTagPoseIsGood || flagVisualSlamIsGood){
    if(submap_id.trajectory_id == 0 && node_id.trajectory_id != 0 ){
      if(mode_constraint == CONSTRAINT_LOCAL && diff_trans > trans_window && !flagTagPoseIsGood && !flagVisualSlamIsGood){
        LOG(WARNING) << " trans_window: " <<  trans_window << " diff_trans:" << diff_trans ;
        return;
      }
      // if(correspondence_cost < 0.6 || correspondence_cost > 1.0){
      //   LOG(WARNING) << "constraint is invalid";
      //   LOG(INFO) << "pose_estimate_global.translation : \n" << pose_estimate_global.translation();
      //   LOG(INFO) << "pose_estimate_global correspondence_cost : \n" << correspondence_cost;
      //   return;
      // }
      if(*scorePoseFromMapPtr_ > 0.6) {
        translation_weight_with_map = translation_weight_with_map / 10.;
        // LOG(INFO) << "*scorePoseFromMapPtr_ high: " << *scorePoseFromMapPtr_;
      }
      else{
        // LOG(INFO) << "*scorePoseFromMapPtr_ low: " << *scorePoseFromMapPtr_;
      }
      constraint->reset(new Constraint{submap_id,
                                      node_id,
                                      {transform::Embed3D(constraint_transform),
                                        translation_weight_with_map * score,
                                        rotation_weight_with_map * score},
                                      Constraint::INTER_SUBMAP});
    }
    else{
        constraint->reset(new Constraint{submap_id,
                                        node_id,
                                        {transform::Embed3D(constraint_transform),
                                          options_.loop_closure_translation_weight() * score,
                                          options_.loop_closure_rotation_weight() * score},
                                        Constraint::INTER_SUBMAP});
    }
  }
  else{
    if(submap_id.trajectory_id == 0 && node_id.trajectory_id != 0){
      if(mode_constraint == CONSTRAINT_LOCAL && diff_trans > trans_window){
        return;
      }
      // if(correspondence_cost < 0.6 || correspondence_cost > 1.0){
      //   LOG(WARNING) << "constraint is invalid";
      //   LOG(INFO) << "pose_estimate_global.translation : \n" << pose_estimate_global.translation();
      //   LOG(INFO) << "pose_estimate_global correspondence_cost : \n" << correspondence_cost;
      //   return;
      // }
      constraint->reset(new Constraint{submap_id,
                                      node_id,
                                      {transform::Embed3D(constraint_transform),
                                        5e4 * score,
                                        5e5 * score},
                                      Constraint::INTER_SUBMAP});
      // LOG(INFO) << "long corridor node_id.node_index: " << node_id.node_index << " with map Constraint score: " << score;
    }
    else{
      constraint->reset(new Constraint{submap_id,
                                      node_id,
                                      {transform::Embed3D(constraint_transform),
                                      // 0.0,
                                        options_.loop_closure_translation_weight() / 100,
                                        options_.loop_closure_rotation_weight()},
                                      Constraint::INTER_SUBMAP});
    }
    if(submap_id.trajectory_id == 0){
    //  LOG(WARNING) << "node_id.node_index: " << node_id.node_index << " with map LongCorridor___ Constraint score: " << score;
    }
  }

  if (globalMessagePtr_->getInitState() == cartographer::InitState::BUILD_CONSTRAINT && initFinish_) {
    globalMessagePtr_->setFlagInitConstraint(true);
    LOG(INFO) << "Check Init Constraint Valid, GOING ...";
  }


  if (options_.log_matches()) {
    std::ostringstream info;
    info << "Node " << node_id << " with "
         << constant_data->filtered_gravity_aligned_point_cloud.size()
         << " points on submap " << submap_id << std::fixed;
    if (mode_constraint != CONSTRAINT_LOCAL) {
      info << " matches";
    } else {
      const transform::Rigid2d difference =
          initial_pose.inverse() * pose_estimate;
      info << " differs by translation " << std::setprecision(2)
           << difference.translation().norm() << " rotation "
           << std::setprecision(3) << std::abs(difference.normalized_angle());
    }
    info << " with score " << std::setprecision(1) << 100. * score << "%.";
    LOG(INFO) << info.str();
  }
}

void ConstraintBuilder2D::RunWhenDoneCallback() {
  Result result;
  std::unique_ptr<std::function<void(const Result&)>> callback;
  {
    absl::MutexLock locker(&mutex_);
    CHECK(when_done_ != nullptr);
    for (const std::unique_ptr<Constraint>& constraint : constraints_) {
      if (constraint == nullptr) continue;
      result.push_back(*constraint);
    }
    if (options_.log_matches()) {
      LOG(INFO) << constraints_.size() << " computations resulted in "
                << result.size() << " additional constraints.";
      LOG(INFO) << "Score histogram:\n" << score_histogram_.ToString(10);
    }
    constraints_.clear();
    callback = std::move(when_done_);
    when_done_.reset();
    kQueueLengthMetric->Set(constraints_.size());
  }
  (*callback)(result);
}

int ConstraintBuilder2D::GetNumFinishedNodes() {
  absl::MutexLock locker(&mutex_);
  return num_finished_nodes_;
}

void ConstraintBuilder2D::DeleteScanMatcher(const SubmapId& submap_id) {
  absl::MutexLock locker(&mutex_);
  if (when_done_) {
    LOG(WARNING)
        << "DeleteScanMatcher was called while WhenDone was scheduled.";
  }
  submap_scan_matchers_.erase(submap_id);
  per_submap_sampler_.erase(submap_id);
  kNumSubmapScanMatchersMetric->Set(submap_scan_matchers_.size());
}

void ConstraintBuilder2D::RegisterMetrics(metrics::FamilyFactory* factory) {
  auto* counts = factory->NewCounterFamily(
      "mapping_constraints_constraint_builder_2d_constraints",
      "Constraints computed");
  kConstraintsSearchedMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "searched"}});
  kConstraintsFoundMetric =
      counts->Add({{"search_region", "local"}, {"matcher", "found"}});
  kGlobalConstraintsSearchedMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "searched"}});
  kGlobalConstraintsFoundMetric =
      counts->Add({{"search_region", "global"}, {"matcher", "found"}});
  auto* queue_length = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_queue_length", "Queue length");
  kQueueLengthMetric = queue_length->Add({});
  auto boundaries = metrics::Histogram::FixedWidth(0.05, 20);
  auto* scores = factory->NewHistogramFamily(
      "mapping_constraints_constraint_builder_2d_scores",
      "Constraint scores built", boundaries);
  kConstraintScoresMetric = scores->Add({{"search_region", "local"}});
  kGlobalConstraintScoresMetric = scores->Add({{"search_region", "global"}});
  auto* num_matchers = factory->NewGaugeFamily(
      "mapping_constraints_constraint_builder_2d_num_submap_scan_matchers",
      "Current number of constructed submap scan matchers");
  kNumSubmapScanMatchersMetric = num_matchers->Add({});
}

}  // namespace constraints
}  // namespace mapping
}  // namespace cartographer
