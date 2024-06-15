#ifndef LQR_STEER_CONTROL
#define LQR_STEER_CONTROL

#include <sys/time.h>

#include <array>
#include <iostream>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace control
{

struct State
{
    double x   = 0.0;
    double y   = 0.0;
    double yaw = 0.0;
    double v   = 0.0;
    double w   = 0.0;
};

class LqrControl
{
public:
    LqrControl(const double control_duration);
    ~LqrControl()
    {
    }

    double Lqr_control_ros(const nav_msgs::msg::Path& global_path, const geometry_msgs::msg::Pose& pose,
                           const geometry_msgs::msg::Twist& speed, const double desired_linear_velocity);

    double Lqr_control(const std::vector<common::CurvePoint>& global_path, const State& state,
                       const double desired_speed);

    void update(State& state, double v, double w);  // for simulation

private:
    /**
     * @brief      Normalize angle to [-PI, PI).
     *
     * @param[in]  angle  The original value of the angle.
     *
     * @return     The normalized value of the angle.
     */
    double normalize_angle(const double angle);

    /**
     * @brief      prediction robot state by time.
     *
     * @param[in]  state  the robot current state.
     * @param[in]  pre_time  the prediction time.
     * @return after the pre_time state
     */
    State prediction_state_by_time(const State state, const double pre_time);

    /**
     * @brief      find nearest path index
     *
     * @param[in]  global_path  the global path.
     * @param[in]  state  the current state
     * @return after the pre_time state
     */
    size_t find_index(const std::vector<common::CurvePoint>& global_path, const State& state);

    /**
     * @brief      calculate lateral deviation
     *
     * @param[in]  global_path  the global path.
     * @param[in]  state  the robot current state
     * @return the lateral deviation
     */
    double calc_lat_deviation(const std::vector<common::CurvePoint>& global_path, const State state);

    /**
     * @brief      calculate heading deviation
     *
     * @param[in]  global_path  the global path.
     * @param[in]  state  the robot current state
     * @return the heading deviation
     */
    double calc_head_deviation(const std::vector<common::CurvePoint>& global_path, const State state);

    /**
     * @brief      calculate heading deviation
     *
     * @param[in]  global_path  the global path.
     * @param[in]  state  the robot current state
     * @return extend the global path when the path is too short
     */
    void extend_path(std::vector<common::CurvePoint>& path);

    Eigen::Matrix4d solve_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, double R);

    Eigen::RowVector4d dlqr(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, double R);

    /**
     * @brief      calculate control command by global path and current pose
     *
     * @param[in]  global_path  the global path.
     * @param[in]  state  the robot current state
     * @return the front wheel angle
     */
    double lqr_steering_control(const std::vector<common::CurvePoint>& global_path, const State state);

    /**
     * @brief      calculate control command by global path and current pose
     *
     * @param[in]  global_path  the global path.
     * @param[in]  state  the robot current state
     * @return the front wheel angle
     */
    double lqr_compute_command(const std::vector<common::CurvePoint>& global_path, const State state);

private:
    size_t nearest_idx_          = 0;      /// the nearest path point index
    double e_last_               = 0;      /// lateral deviation
    double e_th_last_            = 0;      /// heading deviation
    double control_duration_     = 0.05;   /// control duration
    double wheel_base_           = 0.3;    /// the robot wheel base
    double use_prediction_       = false;  // calculate deviation by prediction
    double prediction_time_lat_  = 0.1;    // uinit : s
    double prediction_time_head_ = 0.2;    // uinit : s
    double min_pre_distance_     = 0.15;
    double max_pre_distance_     = 0.4;
};

}  // namespace control

#endif
