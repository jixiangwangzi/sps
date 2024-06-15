#include "nav2_regulated_pure_pursuit_controller/lattice_planner/curve_1d_generator.h"

#include "nav2_regulated_pure_pursuit_controller/math/quintic_polynomial_curve_1d.h"
#include "nav2_regulated_pure_pursuit_controller/util/frame_converter.h"
#include "nav2_regulated_pure_pursuit_controller/util/path_matcher.h"

using namespace math;

namespace lattice_planner
{
Curve1dGenerator::Curve1dGenerator(const CurvePoint& start_pose, const CurvePoint& start_match_point,
                                   const State& init_lon_state, const State& init_lat_state,
                                   const std::vector<CurvePoint>& refer_path, const std::vector<Point>& obstacle_points)
  : start_pose_(start_pose)
  , start_match_point_(start_match_point)
  , init_lon_state_(init_lon_state)
  , init_lat_state_(init_lat_state)
  , refer_path_(refer_path)
  , obstacle_points_(obstacle_points)
{
}
Curve1dGenerator::Curve1dGenerator(const CurvePoint& start_pose, const CurvePoint& start_match_point,
                                   const State& init_lon_state, const State& init_lat_state,
                                   const std::vector<CurvePoint>&     refer_path,
                                   std::shared_ptr<CollisionChecker>& collision_checker)
  : start_pose_(start_pose)
  , start_match_point_(start_match_point)
  , init_lon_state_(init_lon_state)
  , init_lat_state_(init_lat_state)
  , refer_path_(refer_path)
  , collision_checker_(collision_checker)
{
}

void Curve1dGenerator::GenerateCurveSet(double left_road_width, double right_road_width, double vehicle_width,
                                        double lattice_length, Curve1dSet& lat_curve_set)
{
    GenerateLatCurveSet(lat_curve_set, left_road_width, right_road_width, vehicle_width, lattice_length);
}

void Curve1dGenerator::SampleLateralEndConditions(double left_road_width, double right_road_width, double vehicle_width,
                                                  double target_s, std::vector<Condition>& LatEndConditions)
{
    double sample_interval  = 0.1;
    double road_edge_buffer = 0.1;
    double left_boundary    = left_road_width - vehicle_width / 2.0 - road_edge_buffer;
    double right_boundary   = -1.0 * right_road_width + vehicle_width / 2.0 + road_edge_buffer;

    if (left_boundary < 0.0)
    {
        left_boundary = 0.0;
    }

    if (right_boundary > 0.0)
    {
        right_boundary = 0.0;
    }

    for (float sample_l = -1.0 * sample_interval; sample_l > right_boundary; sample_l -= sample_interval)
    {
        State end_lat_state = {sample_l, 0.0, 0.0};
        LatEndConditions.emplace_back(end_lat_state, target_s);
    }

    for (float sample_l = 0.0; sample_l <= left_boundary; sample_l += sample_interval)
    {
        State end_lat_state = {sample_l, 0.0, 0.0};
        LatEndConditions.emplace_back(end_lat_state, target_s);
    }
}

void Curve1dGenerator::GenerateLatCurveSet(Curve1dSet& lat_curve_set, double left_road_width, double right_road_width,
                                           double vehicle_width, double lattice_length)
{
    std::vector<Condition> LatEndConditions;
    SampleLateralEndConditions(left_road_width, right_road_width, vehicle_width, lattice_length, LatEndConditions);

    for (Condition const& end : LatEndConditions)
    {
        std::array<double, 3> s_condition  = {end.second, 0.0, 0.0};
        Point                 sample_point = TransformToCartesian(s_condition, end.first);

        if (!collision_checker_ || collision_checker_->inCollision(sample_point.GetX(), sample_point.GetY()))
        {
            // std::cout << "collision position l : " << end.first.at(0) << std::endl;
            continue;
        }

        // transform start pose state, heading from start point to sample point
        State sample_start_lat_state = init_lat_state_;  // TransformStartPoseStateByTargetPoint(sample_point);
        std::shared_ptr<math::Curve1d> lat_curve =
            std::make_shared<QuinticPolynomialCurve1d>(sample_start_lat_state, end.first, end.second);

        lat_curve_set.push_back(lat_curve);
    }
}

Point Curve1dGenerator::TransformToCartesian(std::array<double, 3> s_condition, std::array<double, 3> l_condition)
{
    Point      point;
    CurvePoint matched_ref_point = PathMatcher::Match2Path(refer_path_, init_lon_state_[0] + s_condition.at(0));

    point.SetX(matched_ref_point.GetX() + l_condition[0] * std::cos(matched_ref_point.GetTheta() + M_PI / 2.0));
    point.SetY(matched_ref_point.GetY() + l_condition[0] * std::sin(matched_ref_point.GetTheta() + M_PI / 2.0));

    // const double rs      = matched_ref_point.GetS();
    // const double rx      = matched_ref_point.GetX();
    // const double ry      = matched_ref_point.GetY();
    // const double rtheta  = matched_ref_point.GetTheta();
    // const double rkappa  = matched_ref_point.GetKappa();
    // const double rdkappa = matched_ref_point.GetDKappa();

    // double x     = 0.0;
    // double y     = 0.0;
    // double theta = 0.0;
    // double kappa = 0.0;
    // double v     = 0.0;
    // double a     = 0.0;

    // FrameConverter::Frenet2Cartesian(rs, rx, ry, rtheta, rkappa, rdkappa, s_condition, l_condition, &x, &y, &theta,
    //                                  &kappa, &v, &a);
    // Point point;
    // point.SetX(x);
    // point.SetY(y);

    return point;
}

std::array<double, 3> Curve1dGenerator::TransformStartPoseStateByTargetPoint(const Point& target_point)
{
    State init_s;
    State init_l;

    double angle_from_start_to_target =
        std::atan2(target_point.GetY() - start_pose_.GetY(), target_point.GetX() - start_pose_.GetX());

    FrameConverter::Cartesian2Frenet(start_match_point_.GetS(), start_match_point_.GetX(), start_match_point_.GetY(),
                                     start_match_point_.GetTheta(), start_match_point_.GetKappa(),
                                     start_match_point_.GetDKappa(), start_pose_.GetX(), start_pose_.GetY(), 0, 0,
                                     angle_from_start_to_target, 0, &init_s, &init_l);
    return init_l;
}

}  // namespace lattice_planner
