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

#include "cartographer/mapping/imu_tracker.h"

#include <cmath>
#include <limits>

#include "cartographer/common/math.h"
#include "cartographer/mapping/internal/eigen_quaterniond_from_two_vectors.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {

ImuTracker::ImuTracker(const double imu_gravity_time_constant,
                       const common::Time time)
    : imu_gravity_time_constant_(imu_gravity_time_constant),
      time_(time),
      last_linear_acceleration_time_(common::Time::min()),
      orientation_(Eigen::Quaterniond::Identity()),
      gravity_vector_(Eigen::Vector3d::UnitZ()),
      imu_angular_velocity_(Eigen::Vector3d::Zero()) {}

void ImuTracker::Advance(const common::Time time) {
  // CHECK_LE(time_, time);
  const double delta_t = common::ToSeconds(time - time_);
  const Eigen::Quaterniond rotation =
      transform::AngleAxisVectorToRotationQuaternion(
          Eigen::Vector3d(imu_angular_velocity_ * delta_t));
  Eigen::Quaterniond orientation_temp = (orientation_ * rotation).normalized();
  Eigen::Vector3d gravity_vector_temp = rotation.conjugate() * gravity_vector_;
  auto orientation_gravity_temp = orientation_temp * gravity_vector_temp;
  if(std::isfinite(orientation_gravity_temp.z())){
    orientation_ = orientation_temp;
    gravity_vector_ = gravity_vector_temp;
  }
  else{
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::Advance() rotation wxyz : \n" << rotation.w() << " " << rotation.x() << " " << rotation.y() << " " <<  rotation.z();
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::Advance() orientation_ wxyz : \n" << orientation_.w() << " " << orientation_.x() << " " << orientation_.y() << " " <<  orientation_.z();
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::Advance() orientation_ : \n" << orientation_.toRotationMatrix();
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::Advance() gravity_vector_  : \n" << gravity_vector_; 
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::Advance() orientation_ * gravity_vector_  : \n" << orientation_ * gravity_vector_; 
  }

  time_ = time;
}

void ImuTracker::AddImuLinearAccelerationObservation(
    const Eigen::Vector3d& imu_linear_acceleration) {
  // Update the 'gravity_vector_' with an exponential moving average using the
  // 'imu_gravity_time_constant'.
  const double delta_t =
   common::ToSeconds(time_ - last_linear_acceleration_time_);
      // last_linear_acceleration_time_ > common::Time::min()
      //     ? common::ToSeconds(time_ - last_linear_acceleration_time_)
      //     : std::numeric_limits<double>::infinity();
  last_linear_acceleration_time_ = time_;
  const double alpha = 1. - std::exp(-delta_t / imu_gravity_time_constant_);
  auto gravity_vector_temp =
      (1. - alpha) * gravity_vector_ + alpha * imu_linear_acceleration;

  // Change the 'orientation_' so that it agrees with the current
  // 'gravity_vector_'.
  const Eigen::Quaterniond rotation = FromTwoVectors(
      gravity_vector_temp, orientation_.conjugate() * Eigen::Vector3d::UnitZ());
  auto orientation_temp = (orientation_ * rotation).normalized();
  auto orientation_gravity_temp = orientation_temp * gravity_vector_temp;
  if(std::isfinite(orientation_gravity_temp.z())){
    gravity_vector_ = gravity_vector_temp;
    orientation_ = orientation_temp;
    if(((orientation_ * gravity_vector_).z() >= 0.) && ((orientation_ * gravity_vector_).normalized().z() >= 0.99)){
      ;
    }
    else{
      LOG_EVERY_N(WARNING, 50000) << "ImuTracker::Advance() rotation wxyz : \n" << rotation.w() << " " << rotation.x() << " " << rotation.y() << " " <<  rotation.z();
      LOG_EVERY_N(WARNING, 50000) << "ImuTracker::AddImuLinearAccelerationObservation() orientation_ wxyz : \n" << orientation_.w() << " " << orientation_.x() << " " << orientation_.y() << " " <<  orientation_.z();
      LOG_EVERY_N(WARNING, 50000) << "ImuTracker::AddImuLinearAccelerationObservation() orientation_ : \n" << orientation_.toRotationMatrix();
      LOG_EVERY_N(WARNING, 50000) << "ImuTracker::AddImuLinearAccelerationObservation() gravity_vector_  : \n" << gravity_vector_; 
      LOG_EVERY_N(WARNING, 50000) << "ImuTracker::AddImuLinearAccelerationObservation() orientation_ * gravity_vector_  : \n" << orientation_ * gravity_vector_; 
    }
  }
  else{
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::Advance() rotation wxyz : \n" << rotation.w() << " " << rotation.x() << " " << rotation.y() << " " <<  rotation.z();
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::AddImuLinearAccelerationObservation() orientation_ wxyz : \n" << orientation_.w() << " " << orientation_.x() << " " << orientation_.y() << " " <<  orientation_.z();
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::AddImuLinearAccelerationObservation() orientation_ : \n" << orientation_.toRotationMatrix();
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::AddImuLinearAccelerationObservation() gravity_vector_  : \n" << gravity_vector_; 
    LOG_EVERY_N(WARNING, 50000) << "ImuTracker::AddImuLinearAccelerationObservation() orientation_ * gravity_vector_  : \n" << orientation_ * gravity_vector_; 
  }
}

void ImuTracker::AddImuAngularVelocityObservation(
    const Eigen::Vector3d& imu_angular_velocity) {
  imu_angular_velocity_ = imu_angular_velocity;
}

}  // namespace mapping
}  // namespace cartographer
