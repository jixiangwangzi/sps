#include "nav2_regulated_pure_pursuit_controller/util/frame_converter.h"

#include "nav2_regulated_pure_pursuit_controller/math/math_util.h"

using namespace math;

namespace util
{
bool FrameConverter::Cartesian2Frenet(const double rs, const double rx, const double ry, const double rtheta,
                                      const double rkappa, const double rdkappa, const double x, const double y,
                                      const double v, const double a, const double theta, const double kappa,
                                      std::array<double, 3>* const ptr_s_condition,
                                      std::array<double, 3>* const ptr_d_condition)
{
    const double dx = x - rx;
    const double dy = y - ry;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    const double cross_rd_nd =
        cos_theta_r * dy - sin_theta_r * dx;  //(cos_theta_r,sin_theta_r)*(dx,dy) = distance of (x,y) to axis s
    ptr_d_condition->at(0) = std::copysign(std::sqrt(dx * dx + dy * dy), cross_rd_nd);

    const double delta_theta     = theta - rtheta;
    const double tan_delta_theta = std::tan(delta_theta);
    const double cos_delta_theta = std::cos(delta_theta);

    const double one_minus_kappa_r_d = 1 - rkappa * ptr_d_condition->at(0);
    ptr_d_condition->at(1)           = one_minus_kappa_r_d * tan_delta_theta;

    const double kappa_r_d_prime = rdkappa * ptr_d_condition->at(0) + rkappa * ptr_d_condition->at(1);

    ptr_d_condition->at(2) =
        -kappa_r_d_prime * tan_delta_theta + one_minus_kappa_r_d / cos_delta_theta / cos_delta_theta *
                                                 (kappa * one_minus_kappa_r_d / cos_delta_theta - rkappa);

    ptr_s_condition->at(0) = rs;

    ptr_s_condition->at(1) = v * cos_delta_theta / one_minus_kappa_r_d;

    const double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * kappa - rkappa;
    ptr_s_condition->at(2) =
        (a * cos_delta_theta - ptr_s_condition->at(1) * ptr_s_condition->at(1) *
                                   (ptr_d_condition->at(1) * delta_theta_prime - kappa_r_d_prime)) /
        one_minus_kappa_r_d;

    return true;
}

bool FrameConverter::Frenet2Cartesian(const double rs, const double rx, const double ry, const double rtheta,
                                      const double rkappa, const double rdkappa,
                                      const std::array<double, 3>& s_condition,
                                      const std::array<double, 3>& d_condition, double* const ptr_x,
                                      double* const ptr_y, double* const ptr_theta, double* const ptr_kappa,
                                      double* const ptr_v, double* const ptr_a)
{
    (void)rs;

    const double cos_theta_r = std::cos(rtheta);
    const double sin_theta_r = std::sin(rtheta);

    *ptr_x = rx - sin_theta_r * d_condition[0];
    *ptr_y = ry + cos_theta_r * d_condition[0];

    const double one_minus_kappa_r_d = 1 - rkappa * d_condition[0];

    const double tan_delta_theta = d_condition[1] / one_minus_kappa_r_d;
    const double delta_theta     = std::atan2(d_condition[1], one_minus_kappa_r_d);
    const double cos_delta_theta = std::cos(delta_theta);

    *ptr_theta = MathUtil::NormalizeAngle(delta_theta + rtheta);

    const double kappa_r_d_prime = rdkappa * d_condition[0] + rkappa * d_condition[1];
    *ptr_kappa = (((d_condition[2] + kappa_r_d_prime * tan_delta_theta) * cos_delta_theta * cos_delta_theta) /
                      (one_minus_kappa_r_d) +
                  rkappa) *
                 cos_delta_theta / (one_minus_kappa_r_d);

    const double d_dot = d_condition[1] * s_condition[1];
    *ptr_v = std::sqrt(one_minus_kappa_r_d * one_minus_kappa_r_d * s_condition[1] * s_condition[1] + d_dot * d_dot);

    const double delta_theta_prime = one_minus_kappa_r_d / cos_delta_theta * (*ptr_kappa) - rkappa;

    *ptr_a = s_condition[2] * one_minus_kappa_r_d / cos_delta_theta +
             s_condition[1] * s_condition[1] / cos_delta_theta * (d_condition[1] * delta_theta_prime - kappa_r_d_prime);

    return true;
}

// void FrameConverter::Global2Body(const Pose2d& global_obj_pos, const Pose2d& global_car_pos, PathPoint&
// local_obj_pose)
// {
//     const double delta_x    = global_obj_pos.GetX() - global_car_pos.GetX();
//     const double delta_y    = global_obj_pos.GetY() - global_car_pos.GetY();
//     const double body_x     = delta_x * cos(global_car_pos.GetTheta()) + delta_y * sin(global_car_pos.GetTheta());
//     const double body_y     = delta_y * cos(global_car_pos.GetTheta()) - delta_x * sin(global_car_pos.GetTheta());
//     const double body_theta = global_obj_pos.GetTheta() - global_car_pos.GetTheta();
//     local_obj_pose.SetX(body_x);
//     local_obj_pose.SetY(body_y);
//     local_obj_pose.SetTheta(body_theta);
// }

// void FrameConverter::Body2Global(const Pose2d& local_obj_pos, const Pose2d& global_car_pos, PathPoint&
// global_obj_pos)
// {
//     const double global_x = local_obj_pos.GetX() * cos(global_car_pos.GetTheta()) -
//                                local_obj_pos.GetY() * sin(global_car_pos.GetTheta()) + global_car_pos.GetX();
//     const double global_y = local_obj_pos.GetX() * sin(global_car_pos.GetTheta()) +
//                                local_obj_pos.GetY() * cos(global_car_pos.GetTheta()) + global_car_pos.GetY();
//     const double global_theta = local_obj_pos.GetTheta() + global_car_pos.GetTheta();
//     global_obj_pos.SetX(global_x);
//     global_obj_pos.SetY(global_y);
//     global_obj_pos.SetTheta(global_theta);
// }

// void FrameConverter::Global2Body(const CurvePoint& global_obj_point, const Pose2d& global_car_pos,
//                                  CurvePoint& local_obj_point)
// {
//     const double local_x = (global_obj_point.GetX() - global_car_pos.GetX()) * cos(global_car_pos.GetTheta()) +
//                               (global_obj_point.GetY() - global_car_pos.GetY()) * sin(global_car_pos.GetTheta());
//     const double local_y = -(global_obj_point.GetX() - global_car_pos.GetX()) * sin(global_car_pos.GetTheta()) +
//                               (global_obj_point.GetY() - global_car_pos.GetY()) * cos(global_car_pos.GetTheta());
//     const double local_theta = global_obj_point.GetTheta() - global_car_pos.GetTheta();
//     local_obj_point.SetX(local_x);
//     local_obj_point.SetY(local_y);
//     local_obj_point.SetTheta(local_theta);
// }

// void FrameConverter::Body2Global(const CurvePoint& local_obj_point, const Pose2d& global_car_pos,
//                                  CurvePoint& global_obj_point)
// {
//     const double global_x = local_obj_point.GetX() * cos(global_car_pos.GetTheta()) -
//                                local_obj_point.GetY() * sin(global_car_pos.GetTheta()) + global_car_pos.GetX();
//     const double global_y = local_obj_point.GetX() * sin(global_car_pos.GetTheta()) +
//                                local_obj_point.GetY() * cos(global_car_pos.GetTheta()) + global_car_pos.GetY();
//     const double global_theta = local_obj_point.GetTheta() + global_car_pos.GetTheta();
//     global_obj_point.SetX(global_x);
//     global_obj_point.SetY(global_y);
//     global_obj_point.SetTheta(global_theta);
// }

}  // namespace util
