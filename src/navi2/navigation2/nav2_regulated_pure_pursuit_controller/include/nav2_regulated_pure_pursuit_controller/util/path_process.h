#ifndef __PATH_PROCESS_H__
#define __PATH_PROCESS_H__

#include <memory>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"
#include "nav2_regulated_pure_pursuit_controller/math/cubic_bezier.h"
#include "nav2_regulated_pure_pursuit_controller/math/spline2d.h"
#include "nav_msgs/msg/path.hpp"

using namespace common;
using namespace math;
using namespace cublic_spline;

namespace util
{
class PathProcess
{
public:
    PathProcess(const double interval);
    PathProcess()  = default;
    ~PathProcess() = default;

    /**
     * @brief      smooth the global path.
     *
     * @param path the input path
     * @param pose the robot current pose
     *
     * @return  the smoothed path.
     */
    std::vector<CurvePoint> smoothPath(const nav_msgs::msg::Path& path, const double smooth_length);

    /**
     * @brief      get offset path
     *
     * @param path the origin path
     * @param offset_distance the offset distance,left is positive and right is negetive
     *
     * @return     the offset path.
     */
    std::vector<CurvePoint> getOffsetPath(const std::vector<CurvePoint>& path, const double offset_distance);

    /**
     * @brief      get offset path
     *
     * @param path the origin path
     * @param offset_distance the offset distance,left is positive and right is negetive
     *
     * @return     the offset path.
     */
    nav_msgs::msg::Path getOffsetPath(const nav_msgs::msg::Path& path, const double offset_distance,
                                      const double extract_max_length);

    /**
     * @brief      find nearest path point index by current pose
     *
     * @param[in]  global_path  the global path points
     * @param[in]  pose  the robot pose
     *
     * @return     the robot frame path
     */
    std::vector<common::CurvePoint> transformGlobalPath(const std::vector<common::CurvePoint>& global_path,
                                                        const size_t start_index, const common::CurvePoint& pose);

    /**
     * @brief      find nearest path point index by current pose
     *
     * @param[in]  global_path  the global path points
     * @param[in]  pose  the robot pose
     *
     * @return     the robot frame path
     */
    std::vector<common::CurvePoint> transformGlobalPath(const std::vector<common::CurvePoint>& global_path,
                                                        const size_t                           start_index,
                                                        const geometry_msgs::msg::PoseStamped& pose);

private:
    size_t searchNearestPathIndex(const nav_msgs::msg::Path& path, const geometry_msgs::msg::PoseStamped& pose);

    std::unique_ptr<CubicBezier> cubic_bezier_;

    double sample_interval_;  // cubic spline sample interval
    double p2p_interval_;
};
}  // namespace util

#endif
