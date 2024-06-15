
#include "nav2_regulated_pure_pursuit_controller/lattice_planner/trajectory_generator.h"

#include "nav2_regulated_pure_pursuit_controller/util/frame_converter.h"
#include "nav2_regulated_pure_pursuit_controller/util/path_matcher.h"

using namespace util;

namespace lattice_planner
{
void TrajectoryGenerator::Generate(const std::vector<CurvePoint>&       ref_path,
                                   const std::shared_ptr<math::Curve1d> lat_curve, const double& start_s,
                                   std::vector<CurvePoint>& trajectory)
{
    assert(false == ref_path.empty());
    assert(lat_curve != nullptr);

    const double s_ref_max = ref_path.back().GetS();

    for (double s = 0.0; s < lat_curve->ParamLength(); s += 0.1)
    {
        std::array<double, 3> s_condition;

        s_condition.at(0) = s;
        if (s_condition.at(0) > s_ref_max)
        {
            break;
        }

        s_condition.at(1) = 0.0;
        s_condition.at(2) = 0.0;

        std::array<double, 3> l_condition;

        l_condition.at(0) = lat_curve->Evaluate(0, s_condition.at(0));
        l_condition.at(1) = lat_curve->Evaluate(1, s_condition.at(0));
        l_condition.at(2) = lat_curve->Evaluate(2, s_condition.at(0));

        CurvePoint matched_ref_point = PathMatcher::Match2Path(ref_path, start_s + s_condition.at(0));

        const double rs      = matched_ref_point.GetS();
        const double rx      = matched_ref_point.GetX();
        const double ry      = matched_ref_point.GetY();
        const double rtheta  = matched_ref_point.GetTheta();
        const double rkappa  = matched_ref_point.GetKappa();
        const double rdkappa = matched_ref_point.GetDKappa();

        double x     = 0.0;
        double y     = 0.0;
        double theta = 0.0;
        double kappa = 0.0;
        double v     = 0.0;
        double a     = 0.0;

        FrameConverter::Frenet2Cartesian(rs, rx, ry, rtheta, rkappa, rdkappa, s_condition, l_condition, &x, &y, &theta,
                                         &kappa, &v, &a);

        CurvePoint pt;
        pt.SetX(x);
        pt.SetY(y);
        pt.SetZ(0.0);
        pt.SetTheta(theta);
        pt.SetKappa(kappa);
        pt.SetS(start_s + s_condition.at(0));

        trajectory.push_back(std::move(pt));
    }
}

}  // namespace lattice_planner
