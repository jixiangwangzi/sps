/*
 * Copyright 2017 The Cartographer Authors
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

#include "cartographer/mapping/pose_extrapolator.h"

#include <algorithm>

#include "absl/memory/memory.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

PoseExtrapolator::PoseExtrapolator(const common::Duration pose_queue_duration,
                                   double imu_gravity_time_constant)
    : pose_queue_duration_(pose_queue_duration),
      gravity_time_constant_(imu_gravity_time_constant),
      cached_extrapolated_pose_{common::Time::min(),
                                transform::Rigid3d::Identity()} {}

std::unique_ptr<PoseExtrapolator> PoseExtrapolator::InitializeWithImu(
    const common::Duration pose_queue_duration,
    const double imu_gravity_time_constant, const sensor::ImuData& imu_data) {
  auto extrapolator = absl::make_unique<PoseExtrapolator>(
      pose_queue_duration, imu_gravity_time_constant);
  extrapolator->AddImuData(imu_data);
  extrapolator->imu_tracker_ =
      absl::make_unique<ImuTracker>(imu_gravity_time_constant, imu_data.time);
  extrapolator->imu_tracker_->AddImuLinearAccelerationObservation(
      imu_data.linear_acceleration);
  extrapolator->imu_tracker_->AddImuAngularVelocityObservation(
      imu_data.angular_velocity);
  extrapolator->imu_tracker_->Advance(imu_data.time);
  extrapolator->AddPose(
      imu_data.time,
      transform::Rigid3d::Rotation(extrapolator->imu_tracker_->orientation()));
  return extrapolator;
}

common::Time PoseExtrapolator::GetLastPoseTime() const {
  if (timed_pose_queue_.empty()) {
    return common::Time::min();
  }
  return timed_pose_queue_.back().time;
}

common::Time PoseExtrapolator::GetLastExtrapolatedTime() const {
  if (!extrapolation_imu_tracker_) {
    return common::Time::min();
  }
  return extrapolation_imu_tracker_->time();
}

void PoseExtrapolator::AddPose(const common::Time time,
                               const transform::Rigid3d& pose) {
  if (imu_tracker_ == nullptr) {
    common::Time tracker_start = time;
    if (!imu_data_.empty()) {
      tracker_start = std::min(tracker_start, imu_data_.front().time);
    }
    imu_tracker_ =
        absl::make_unique<ImuTracker>(gravity_time_constant_, tracker_start);
  }
  timed_pose_queue_.push_back(TimedPose{time, pose});
  if(odometry_data_.size() > 1){
    transform::Rigid3d base_newest_to_odom = odometry_data_.back().pose;
    odom_to_newest_pose_ = base_newest_to_odom.inverse();
    flag_init_odom_to_newest_pose_ = true;
  }
  while (timed_pose_queue_.size() > 2 &&
         timed_pose_queue_[1].time <= time - pose_queue_duration_) {
    timed_pose_queue_.pop_front();
  }
  UpdateVelocitiesFromPoses();
  AdvanceImuTracker(time, imu_tracker_.get());
  TrimImuData();
  TrimOdometryData();
  odometry_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
  extrapolation_imu_tracker_ = absl::make_unique<ImuTracker>(*imu_tracker_);
}

void PoseExtrapolator::AddImuData(const sensor::ImuData& imu_data) {
  CHECK(timed_pose_queue_.empty() ||
        imu_data.time >= timed_pose_queue_.back().time);
  imu_data_.push_back(imu_data);
  TrimImuData();
}

void PoseExtrapolator::AddOdometryData(
    const sensor::OdometryData& odometry_data) {
  CHECK(timed_pose_queue_.empty() ||
        odometry_data.time >= timed_pose_queue_.back().time);
  odometry_data_.push_back(odometry_data);
  TrimOdometryData();
  if (odometry_data_.size() < 2) {
    return;
  }
  // TODO(whess): Improve by using more than just the last two odometry poses.
  // Compute extrapolation in the tracking frame.
  const sensor::OdometryData& odometry_data_oldest = odometry_data_.front();
  const sensor::OdometryData& odometry_data_newest = odometry_data_.back();
  const double odometry_time_delta =
      common::ToSeconds(odometry_data_oldest.time - odometry_data_newest.time);
  const transform::Rigid3d odometry_pose_delta =
      odometry_data_newest.pose.inverse() * odometry_data_oldest.pose;
  Eigen::Vector3d temp_angular_velocity_from_odometry_ =
      transform::RotationQuaternionToAngleAxisVector(
          odometry_pose_delta.rotation()) /
      odometry_time_delta;
  if (timed_pose_queue_.empty()) {
    return;
  }
  const Eigen::Vector3d
      linear_velocity_in_tracking_frame_at_newest_odometry_time =
          odometry_data_.back().line_velocity;
  const Eigen::Quaterniond orientation_at_newest_odometry_time =
      timed_pose_queue_.back().pose.rotation() *
      ExtrapolateRotation(odometry_data_newest.time,
                          odometry_imu_tracker_.get());
  Eigen::Vector3d temp_linear_velocity_from_odometry_  =
      orientation_at_newest_odometry_time * 
      linear_velocity_in_tracking_frame_at_newest_odometry_time;
  // temp_linear_velocity_from_odometry_  = odometry_data_.back().line_velocity;
  temp_angular_velocity_from_odometry_ = odometry_data_.back().angular_velocity;

  if(temp_linear_velocity_from_odometry_.norm() < 3.0 && temp_angular_velocity_from_odometry_.norm() < 3.0){
    linear_velocity_from_odometry_ = temp_linear_velocity_from_odometry_;
    angular_velocity_from_odometry_ = temp_angular_velocity_from_odometry_ ;
  }
  else{
    LOG(INFO) << "odometry_data_.front(): \n" << odometry_data_.front().pose;
    LOG(INFO) << "odometry_data_.back(): \n" << odometry_data_.back().pose;
    LOG(INFO) << "temp_linear_velocity_from_odometry_: \n" << temp_linear_velocity_from_odometry_ << "\n \n norrm: " << temp_linear_velocity_from_odometry_.norm();
    LOG(INFO) << "temp_angular_velocity_from_odometry_: \n" << temp_angular_velocity_from_odometry_ << "\n \n norrm: " << temp_angular_velocity_from_odometry_.norm();
  }
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  // CHECK_GE(time, newest_timed_pose.time);
  // LOG(INFO) << "cached_extrapolated_pose_.time" << cached_extrapolated_pose_.time << " time: " << time;
  if (cached_extrapolated_pose_.time != time) {
    // if(flag_init_odom_to_newest_pose_){
    //   transform::Rigid3d pose = newest_timed_pose.pose * odom_to_newest_pose_ * odometry_data_.back().pose;
    //   LOG(INFO) << "newest_timed_pose.pose: " << newest_timed_pose.pose.translation().x();
    //   LOG(INFO) << "odom_to_newest_pose_: " << odom_to_newest_pose_.translation().x();
    //   LOG(INFO) << "odometry_data_.back().pose: " << odometry_data_.back().pose.translation().x();
    //   LOG(INFO) << "pose: " << pose.translation().x();
    //   cached_extrapolated_pose_ = TimedPose{time, pose};
    // }
    // else{
      const Eigen::Vector3d translation =
          ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
      const Eigen::Quaterniond rotation =
          newest_timed_pose.pose.rotation() *
          ExtrapolateRotation(time, extrapolation_imu_tracker_.get());    
      cached_extrapolated_pose_ =
          TimedPose{time, transform::Rigid3d{translation, rotation}};
    // }
  }
  return cached_extrapolated_pose_.pose;
}

transform::Rigid3d PoseExtrapolator::ExtrapolatePoseFromPose(const common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  // CHECK_GE(time, newest_timed_pose.time);
  // LOG(INFO) << "cached_extrapolated_pose_.time" << cached_extrapolated_pose_.time << " time: " << time;
  // if (cached_extrapolated_pose_.time != time) {
    if(flag_init_odom_to_newest_pose_){
      transform::Rigid3d pose = newest_timed_pose.pose * odom_to_newest_pose_ * odometry_data_.back().pose;
      // LOG(INFO) << "newest_timed_pose.pose: " << newest_timed_pose.pose.translation().y();
      // LOG(INFO) << "odom_to_newest_pose_: " << odom_to_newest_pose_.translation().y();
      // LOG(INFO) << "odometry_data_.back().pose: " << odometry_data_.back().pose.translation().y();
      // LOG(INFO) << "pose: " << pose.translation().y();
      cached_extrapolated_pose_ = TimedPose{time, pose};
    }
    else{
      const Eigen::Vector3d translation =
          ExtrapolateTranslation(time) + newest_timed_pose.pose.translation();
      const Eigen::Quaterniond rotation =
          newest_timed_pose.pose.rotation() *
          ExtrapolateRotation(time, extrapolation_imu_tracker_.get());    
      cached_extrapolated_pose_ =
          TimedPose{time, transform::Rigid3d{translation, rotation}};
    }

  // }
  return cached_extrapolated_pose_.pose;
}

Eigen::Quaterniond PoseExtrapolator::EstimateGravityOrientation(
    const common::Time time) {
  ImuTracker imu_tracker = *imu_tracker_;
  AdvanceImuTracker(time, &imu_tracker);
  return Eigen::Quaterniond::Identity();
}

void PoseExtrapolator::UpdateVelocitiesFromPoses() {
  if (timed_pose_queue_.size() < 2) {
    // We need two poses to estimate velocities.
    return;
  }
  CHECK(!timed_pose_queue_.empty());
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const auto newest_time = newest_timed_pose.time;
  const TimedPose& oldest_timed_pose = timed_pose_queue_.front();
  const auto oldest_time = oldest_timed_pose.time;
  const double queue_delta = common::ToSeconds(newest_time - oldest_time);
  if (queue_delta < common::ToSeconds(pose_queue_duration_)) {
    LOG(WARNING) << "Queue too short for velocity estimation. Queue duration: "
                 << queue_delta << " s";
    return;
  }
  const transform::Rigid3d& newest_pose = newest_timed_pose.pose;
  const transform::Rigid3d& oldest_pose = oldest_timed_pose.pose;
  linear_velocity_from_poses_ =
      (newest_pose.translation() - oldest_pose.translation()) / queue_delta;
  angular_velocity_from_poses_ =
      transform::RotationQuaternionToAngleAxisVector(
          oldest_pose.rotation().inverse() * newest_pose.rotation()) /
      queue_delta;
}

void PoseExtrapolator::TrimImuData() {
  while (imu_data_.size() > 1 && !timed_pose_queue_.empty() &&
         imu_data_[1].time <= timed_pose_queue_.back().time) {
    imu_data_.pop_front();
  }
}

void PoseExtrapolator::TrimOdometryData() {
  // std::cout << "PoseExtrapolator::TrimOdometryData()" << std::endl;
  while (odometry_data_.size() > 2 
        // && !timed_pose_queue_.empty() 
        // && odometry_data_[1].time <= timed_pose_queue_.back().time
        ) {
    odometry_data_.pop_front();
    // std::cout << "PoseExtrapolator:: odometry_data_.pop_front()()" << std::endl;

  }
}

void PoseExtrapolator::AdvanceImuTracker(const common::Time time,
                                         ImuTracker* const imu_tracker) const {
  // CHECK_GE(time, imu_tracker->time());
  if (imu_data_.empty() || time < imu_data_.front().time) {
    // There is no IMU data until 'time', so we advance the ImuTracker and use
    // the angular velocities from poses and fake gravity to help 2D stability.
    imu_tracker->Advance(time);
    imu_tracker->AddImuLinearAccelerationObservation(Eigen::Vector3d::UnitZ());
    imu_tracker->AddImuAngularVelocityObservation(
        odometry_data_.size() < 2 ? angular_velocity_from_poses_
                                  : angular_velocity_from_odometry_);
    return;
  }
  if (imu_tracker->time() < imu_data_.front().time) {
    // Advance to the beginning of 'imu_data_'.
    imu_tracker->Advance(imu_data_.front().time);
  }
  auto it = std::lower_bound(
      imu_data_.begin(), imu_data_.end(), imu_tracker->time(),
      [](const sensor::ImuData& imu_data, const common::Time& time) {
        return imu_data.time < time;
      });
  while (it != imu_data_.end() && it->time < time) {
    imu_tracker->Advance(it->time);
    imu_tracker->AddImuLinearAccelerationObservation(it->linear_acceleration);
    imu_tracker->AddImuAngularVelocityObservation(it->angular_velocity);
    ++it;
  }
  imu_tracker->Advance(time);
}

Eigen::Quaterniond PoseExtrapolator::ExtrapolateRotation(
    const common::Time time, ImuTracker* const imu_tracker) const {
  // CHECK_GE(time, imu_tracker->time());
  AdvanceImuTracker(time, imu_tracker);
  const Eigen::Quaterniond last_orientation = imu_tracker_->orientation();
  return last_orientation.inverse() * imu_tracker->orientation();
}

Eigen::Vector3d PoseExtrapolator::ExtrapolateTranslation(common::Time time) {
  const TimedPose& newest_timed_pose = timed_pose_queue_.back();
  const double extrapolation_delta =
      common::ToSeconds(time - newest_timed_pose.time);
  if (odometry_data_.size() < 2) {
    // std::cout << "----odometry_data_.size(): " << odometry_data_.size() << std::endl;
    return extrapolation_delta * linear_velocity_from_poses_;
  }
  return extrapolation_delta * linear_velocity_from_odometry_ * 1.00;
}

PoseExtrapolator::ExtrapolationResult
PoseExtrapolator::ExtrapolatePosesWithGravity(
    const std::vector<common::Time>& times) {
  std::vector<transform::Rigid3f> poses;
  for (auto it = times.begin(); it != std::prev(times.end()); ++it) {
    poses.push_back(ExtrapolatePose(*it).cast<float>());
  }

  const Eigen::Vector3d current_velocity = odometry_data_.size() < 2
                                               ? linear_velocity_from_poses_
                                               : linear_velocity_from_odometry_;
  return ExtrapolationResult{poses, ExtrapolatePose(times.back()),
                             current_velocity,
                             EstimateGravityOrientation(times.back())};
}

}  // namespace mapping
}  // namespace cartographer