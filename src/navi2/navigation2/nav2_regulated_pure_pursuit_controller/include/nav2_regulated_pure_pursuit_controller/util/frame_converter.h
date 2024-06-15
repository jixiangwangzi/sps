/*!
 *  \brief
 *  \author sunlei (sunlei@holomatic.com)
 *  \date 2018-10-30
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#ifndef _FRAME_CONVERTER_H_
#define _FRAME_CONVERTER_H_

#include <array>
#include <cmath>

namespace util
{
class FrameConverter
{
public:
    FrameConverter() = delete;

    static bool Cartesian2Frenet(const double rs, const double rx, const double ry, const double rtheta,
                                 const double rkappa, const double rdkappa, const double x, const double y,
                                 const double v, const double a, const double theta, const double kappa,
                                 std::array<double, 3>* const ptr_s_condition,
                                 std::array<double, 3>* const ptr_d_condition);

    static bool Frenet2Cartesian(const double rs, const double rx, const double ry, const double rtheta,
                                 const double rkappa, const double rdkappa, const std::array<double, 3>& s_condition,
                                 const std::array<double, 3>& d_condition, double* const ptr_x, double* const ptr_y,
                                 double* const ptr_theta, double* const ptr_kappa, double* const ptr_v,
                                 double* const ptr_a);

    // static void Global2Body(const Pose2d& global_obj_pos, const Pose2d& global_car_pos, PathPoint& local_obj_pose);

    // static void Body2Global(const Pose2d& local_obj_pos, const Pose2d& global_car_pos, PathPoint& global_obj_pos);

    // static void Global2Body(const CurvePoint& global_obj_point, const Pose2d& global_car_pos,
    //                         CurvePoint& local_obj_point);

    // static void Body2Global(const CurvePoint& local_obj_point, const Pose2d& global_car_pos,
    //                         CurvePoint& global_obj_point);
};

}  // namespace util

#endif
