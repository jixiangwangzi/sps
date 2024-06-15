#include "nav2_regulated_pure_pursuit_controller/lattice_planner/curve_evaluator.h"

#include "nav2_regulated_pure_pursuit_controller/util/path_matcher.h"

namespace lattice_planner
{
void CurveEvaluator::Evaluate(const Curve1dSet& lat_curve_set)
{
    for (std::shared_ptr<math::Curve1d> const& lat_curve : lat_curve_set)
    {
        const double lat_cost = CalLatCost(lat_curve);

        cost_queue_.emplace(lat_curve, lat_cost);
    }
}

double CurveEvaluator::CalLatCost(const std::shared_ptr<math::Curve1d>& ptr_lat_curve) const
{
    assert(ptr_lat_curve != nullptr);

    double       max_s               = ptr_lat_curve->ParamLength();
    double       cost_lat_offset_sum = 0.0;
    const double lat_offset_start_l  = ptr_lat_curve->Evaluate(0, 0.0);

    for (double s = 0.0; s <= max_s; s += 0.5)
    {
        double lat_offset = ptr_lat_curve->Evaluate(0, std::min(ptr_lat_curve->ParamLength(), s));

        if (fabs(lat_offset) > 0.5 && lat_offset * lat_offset_start_l < 0.0)
        {
            cost_lat_offset_sum += fabs(lat_offset) * 20;
        }
        else
        {
            cost_lat_offset_sum += fabs(lat_offset) * 1.0;
        }
    }

    return cost_lat_offset_sum;
}
}  // namespace lattice_planner
