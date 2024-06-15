#ifndef __PURE_PURSUIT_CONTROL_H__
#define __PURE_PURSUIT_CONTROL_H__

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"
#include "nav2_regulated_pure_pursuit_controller/util/path_process.h"

using namespace common;
using namespace util;

namespace control
{

class PurePursuitControl
{
public:
    PurePursuitControl(const double& min_lookahead_distance, const double& max_lookahead_distance);
    ~PurePursuitControl() = default;

    double CalcuControlCmd(const std::vector<CurvePoint>& global_path, const geometry_msgs::msg::PoseStamped& cur_pose,
                           const double& cur_velocity, const double& desire_velocity, const double pre_time);

    CurvePoint getLookAheadPoint(const std::vector<CurvePoint>&         global_path,
                                 const geometry_msgs::msg::PoseStamped& cur_pose, const double& cur_velocity,
                                 const double pre_time);
    double     CalcuControlCmd(const CurvePoint& lookahead_point, const double& desire_velocity);

private:
    /**
     * @brief      calculate look ahead distance by velocity and preview time
     *
     * @param[in]  linear_velocity  the linear velocity
     *
     * @return     the lookahead distance
     */
    double calculateLookAheadDistance(const double& linear_velocity, const double& pre_time);

    /**
     * @brief      search the lookahead point by lookahead distance
     *
     * @param[in]  robot_path  the robot frame path
     * @param[in]  lookahead_distance  the lookahead distance
     *
     * @return     the lookahead point
     */
    common::CurvePoint searchLookAheadPoint(const std::vector<common::CurvePoint>& robot_path,
                                            const double                           lookahead_distance);

private:
    size_t searchNearestIndex(const std::vector<CurvePoint>& global_path, const geometry_msgs::msg::PoseStamped& pose);

    std::unique_ptr<PathProcess> path_process_;

    double min_lookahead_distance_;
    double max_lookahead_distance_;
};
}  // namespace control
#endif
