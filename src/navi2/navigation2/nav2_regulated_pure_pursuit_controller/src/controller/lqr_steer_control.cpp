#include "nav2_regulated_pure_pursuit_controller/controller/lqr_steer_control.h"

#include <algorithm>

#include "nav2_regulated_pure_pursuit_controller/math/spline2d.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"
#include "tf2/utils.h"

// using namespace lqr_control;

namespace control
{
LqrControl::LqrControl(const double control_duration) : control_duration_(control_duration)
{
}

double LqrControl::Lqr_control_ros(const nav_msgs::msg::Path& global_path, const geometry_msgs::msg::Pose& pose,
                                   const geometry_msgs::msg::Twist& speed, const double desired_linear_velocity)
{
    if (global_path.poses.empty())
    {
        return 0.0;
    }

    // transform path format
    std::vector<float> array_x;
    std::vector<float> array_y;
    double             sample_interval_square = 0.4 * 0.4;

    array_x.emplace_back(global_path.poses[0].pose.position.x);
    array_y.emplace_back(global_path.poses[0].pose.position.y);

    for (size_t index = 1; index < global_path.poses.size(); ++index)
    {
        double delta_x         = array_x.back() - global_path.poses[index].pose.position.x;
        double delta_y         = array_y.back() - global_path.poses[index].pose.position.y;
        double distance_square = delta_x * delta_x + delta_y * delta_y;

        if (distance_square > sample_interval_square)
        {
            array_x.emplace_back(global_path.poses[index].pose.position.x);
            array_y.emplace_back(global_path.poses[index].pose.position.y);
        }
    }

    double path_end_heading = tf2::getYaw(global_path.poses.back().pose.orientation);

    if (array_x.size() < 20)
    {
        for (double s = 0.4; s < 6.0; s += 0.4)
        {
            array_x.emplace_back(s * std::cos(path_end_heading));
            array_y.emplace_back(s * std::sin(path_end_heading));
        }
    }

    std::vector<common::CurvePoint> processed_path;
    cublic_spline::Spline2D         spline2d(array_x, array_y);
    common::CurvePoint              path_point;

    for (float i = 0; i < spline2d.s.back(); i += 0.05)
    {
        std::array<float, 2> point = spline2d.calc_postion(i);
        path_point.SetS(static_cast<double>(i));
        path_point.SetX(static_cast<double>(point[0]));
        path_point.SetY(static_cast<double>(point[1]));
        path_point.SetTheta(static_cast<double>(spline2d.calc_yaw(i)));
        path_point.SetKappa(static_cast<double>(spline2d.calc_curvature(i)));

        processed_path.emplace_back(path_point);
    }
    // set pose value
    State state;
    state.x   = pose.position.x;
    state.y   = pose.position.y;
    state.yaw = tf2::getYaw(pose.orientation);
    state.v   = speed.linear.x;
    state.w   = speed.angular.z;

    return LqrControl::Lqr_control(processed_path, state, desired_linear_velocity);
}

double LqrControl::Lqr_control(const std::vector<common::CurvePoint>& global_path, const State& state,
                               const double desired_speed)
{
    double                          steer_angle = 0;
    std::vector<common::CurvePoint> plan_path   = global_path;

    nearest_idx_ = find_index(plan_path, state);

    if (plan_path.back().GetS() - plan_path[nearest_idx_].GetS() < 3.0)
    {
        extend_path(plan_path);
    }

    // steer_angle = lqr_steering_control(plan_path, state);
    steer_angle = lqr_compute_command(plan_path, state);

    return (desired_speed * std::tan(steer_angle) / wheel_base_);
}

size_t LqrControl::find_index(const std::vector<common::CurvePoint>& global_path, const State& state)
{
    double min_distance = std::numeric_limits<double>::max();
    size_t key          = 0;

    for (size_t index = 0; index < global_path.size(); index++)
    {
        double distance = std::hypot(state.x - global_path[index].GetX(), state.y - global_path[index].GetY());

        if (min_distance > distance)
        {
            min_distance = distance;
            key          = index;
        }
    }
    return key;
}

double LqrControl::calc_lat_deviation(const std::vector<common::CurvePoint>& global_path, const State state)
{
    double min_distance = std::numeric_limits<double>::max();
    State  cur_state    = state;
    size_t near_index   = nearest_idx_;

    if (use_prediction_)
    {
        cur_state = prediction_state_by_time(state, prediction_time_lat_);

        for (unsigned int index = nearest_idx_; index < global_path.size(); index++)
        {
            double delta_x         = global_path[index].GetX() - cur_state.x;
            double delta_y         = global_path[index].GetY() - cur_state.y;
            double distance_square = delta_x * delta_x + delta_y * delta_y;

            if (distance_square < min_distance)
            {
                min_distance = distance_square;
                near_index   = index;
            }
        }
        // min_distance = std::sqrt(min_distance);
    }
    // else
    // {
    //     min_distance =
    //         std::hypot(global_path[near_index].GetX() - cur_state.x, global_path[near_index].GetY() - cur_state.y);
    // }

    // double dxl   = global_path[near_index].GetX() - cur_state.x;
    // double dyl   = global_path[near_index].GetY() - cur_state.y;
    // double angle = normalize_angle(global_path[near_index].GetTheta() - std::atan2(dyl, dxl));

    // if (angle < 0)
    // {
    //     min_distance = min_distance * -1.0;
    // }

    const double dx                 = cur_state.x - global_path[near_index].GetX();
    const double dy                 = cur_state.y - global_path[near_index].GetY();
    const double cos_target_heading = std::cos(global_path[near_index].GetTheta());
    const double sin_target_heading = std::sin(global_path[near_index].GetTheta());

    min_distance = cos_target_heading * dy - sin_target_heading * dx;

    return min_distance;
}

void LqrControl::extend_path(std::vector<common::CurvePoint>& path)
{
    for (float l = 0.1; l < 3.0; l += 0.1)
    {
        common::CurvePoint pt = path.back();
        pt.SetS(pt.GetS() + l);
        pt.SetX(pt.GetX() + l * std::cos(pt.GetTheta()));
        pt.SetY(pt.GetY() + l * std::sin(pt.GetTheta()));

        path.emplace_back(pt);
    }
}

Eigen::Matrix4d LqrControl::solve_DARE(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, double R)
{
    Eigen::Matrix4d X       = Q;
    int             maxiter = 150;
    double          eps     = 0.01;

    for (int i = 0; i < maxiter; i++)
    {
        Eigen::Matrix4d Xn =
            A.transpose() * X * A - A.transpose() * X * B / (R + B.transpose() * X * B) * B.transpose() * X * A + Q;
        Eigen::Matrix4d error = Xn - X;
        if (error.cwiseAbs().maxCoeff() < eps)
        {
            return Xn;
        }
        X = Xn;
    }

    return X;
}

Eigen::RowVector4d LqrControl::dlqr(Eigen::Matrix4d A, Eigen::Vector4d B, Eigen::Matrix4d Q, double R)
{
    Eigen::Matrix4d    X = solve_DARE(A, B, Q, R);
    Eigen::RowVector4d K = 1.0 / (B.transpose() * X * B + R) * (B.transpose() * X * A);
    return K;
}

double LqrControl::lqr_compute_command(const std::vector<common::CurvePoint>& global_path, const State state)
{
    double          steer = 0.0;
    Eigen::MatrixXd Q     = Eigen::MatrixXd::Zero(3, 3);
    Eigen::MatrixXd R     = Eigen::MatrixXd::Zero(1, 1);
    Q << 1, 0, 0, 0, 1, 0, 0, 0, 1;
    R << 1;

    double curvature = global_path[nearest_idx_].GetKappa();
    if (state.v < 0)
    {
        curvature = -curvature;
    }

    double feed_forword = std::atan2(wheel_base_ * curvature, 1.0);

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
    A(0, 0)           = 1.0;
    A(0, 2)           = -state.v * std::sin(global_path[nearest_idx_].GetTheta()) * control_duration_;
    A(1, 1)           = 1;
    A(1, 2)           = state.v * std::cos(global_path[nearest_idx_].GetTheta()) * control_duration_;
    A(2, 2)           = 1.0;

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 1);
    B(2, 0)           = state.v * control_duration_ / wheel_base_ / std::pow(std::cos(feed_forword), 2);

    double          delta_x   = state.x - global_path[nearest_idx_].GetX();
    double          delta_y   = state.y - global_path[nearest_idx_].GetY();
    double          delta_yaw = normalize_angle(state.yaw - global_path[nearest_idx_].GetTheta());
    Eigen::VectorXd dx(3);
    dx << delta_x, delta_y, delta_yaw;

    double eps  = 0.01;
    double diff = std::numeric_limits<double>::max();

    Eigen::MatrixXd P        = Q;
    Eigen::MatrixXd AT       = A.transpose();
    Eigen::MatrixXd BT       = B.transpose();
    int             num_iter = 0;
    int             maxiter  = 150;
    while (num_iter++ < maxiter && diff > eps)
    {
        Eigen::MatrixXd Pn = AT * P * A - AT * P * B * (R + BT * P * B).inverse() * BT * P * A + Q;
        diff               = ((Pn - P).array().abs()).maxCoeff();
        P                  = Pn;
    }

    Eigen::MatrixXd feed_back = -((R + BT * P * B).inverse() * BT * P * A) * dx;
    steer                     = normalize_angle(feed_back(0, 0) + feed_forword);

    return steer;
}

double LqrControl::lqr_steering_control(const std::vector<common::CurvePoint>& global_path, const State state)
{
    double e    = calc_lat_deviation(global_path, state);
    double th_e = calc_head_deviation(global_path, state);  // left is positive,right is negative

    double k = global_path[nearest_idx_].GetKappa();

    Eigen::Matrix4d A = Eigen::Matrix4d::Zero();
    A(0, 0)           = 1.0;
    A(0, 1)           = control_duration_;
    A(1, 2)           = state.v;
    A(2, 2)           = 1.0;
    A(2, 3)           = control_duration_;

    Eigen::Vector4d B = Eigen::Vector4d::Zero();
    B(3)              = state.v / wheel_base_;

    Eigen::Matrix4d Q = Eigen::Matrix4d::Identity();
    // Q(0, 0)           = 10.0;
    // Q(0, 0)           = 10.0;
    // Q(2, 2)  = 10.0;
    double R = 1;

    // gain of lqr
    Eigen::RowVector4d K = dlqr(A, B, Q, R);

    Eigen::Vector4d x = Eigen::Vector4d::Zero();
    x(0)              = e;
    x(1)              = (e - e_last_) / control_duration_;  // or state.v * std::sin(th_e);
    x(2)              = th_e;
    x(3)              = (th_e - e_th_last_) / control_duration_;  // or state.w - state.v / k;

    double ff    = std::atan2((wheel_base_ * k), (double)1.0);
    double fb    = normalize_angle((-K * x)(0));
    double delta = ff + fb;

    e_last_    = e;
    e_th_last_ = th_e;

    // std::cout << "lateral diff and angle diff , " << e << "," << th_e << std::endl;
    return delta;
}
void LqrControl::update(State& state, double v, double w)
{
    state.x += v * control_duration_ * std::cos(state.yaw);
    state.y += v * control_duration_ * std::sin(state.yaw);
    state.yaw += w * control_duration_;
    state.w = w;
    state.v = v;
}

double LqrControl::normalize_angle(const double angle)  //[-M_PI,M_PI]
{
    double a = std::fmod(angle + M_PI, 2.0 * M_PI);

    if (a < 0.0)
    {
        a += (2.0 * M_PI);
    }
    return a - M_PI;
}

State LqrControl::prediction_state_by_time(const State state, const double pre_time)
{
    State state_pre = state;
    int   i         = 1;

    while (true)
    {
        if (static_cast<double>(i++) * control_duration_ > pre_time)
        {
            break;
        }

        state_pre.x += state_pre.v * control_duration_ * std::cos(state_pre.yaw);
        state_pre.y += state_pre.v * control_duration_ * std::sin(state_pre.yaw);
        state_pre.yaw += state_pre.w * control_duration_;
    }

    return state_pre;
}
double LqrControl::calc_head_deviation(const std::vector<common::CurvePoint>& global_path, const State state)
{
    size_t nearest_idx  = nearest_idx_;
    double pre_distance = state.v * prediction_time_head_;

    pre_distance = std::clamp(pre_distance, min_pre_distance_, max_pre_distance_);

    if (use_prediction_)
    {
        for (size_t index = nearest_idx_; index < global_path.size(); index++)
        {
            if ((global_path[index].GetS() - global_path[nearest_idx_].GetS()) > pre_distance)
            {
                break;
            }
            else
            {
                nearest_idx = index;
            }
        }
    }

    return normalize_angle(state.yaw - global_path[nearest_idx].GetTheta());
}

}  // namespace control
