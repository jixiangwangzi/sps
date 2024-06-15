
#ifndef __LATTICE_TRAJECTORY_GENERATOR_H__
#define __LATTICE_TRAJECTORY_GENERATOR_H__

#include <assert.h>

#include <memory>

#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"
#include "nav2_regulated_pure_pursuit_controller/math/curve_1d.h"

using namespace common;

namespace lattice_planner
{
/**
 * @brief Class for generate 2d trajectory
 */
class TrajectoryGenerator
{
public:
    /**
     * @brief delete constructor
     */
    TrajectoryGenerator() = delete;

    /**
     * @brief combine longitudinal and lateral curves into 2D path
     *
     * @param ref_path reference path
     * @param lon_curve longitudinal curve
     * @param lat_curve lateral curve
     * @param trajectory generated 2D path
     */
    static void Generate(const std::vector<CurvePoint>& ref_path, const std::shared_ptr<math::Curve1d> lat_curve,
                         const double& start_s, std::vector<CurvePoint>& trajectory);
};

}  // namespace lattice_planner

#endif
