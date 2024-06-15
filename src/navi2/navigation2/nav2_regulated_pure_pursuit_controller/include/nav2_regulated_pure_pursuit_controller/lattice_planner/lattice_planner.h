#ifndef _LATTICE_PLANNER_H_
#define _LATTICE_PLANNER_H_
#include <assert.h>

#include <array>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_regulated_pure_pursuit_controller/util/collision_checker.hpp"
#include "tf2/utils.h"

using namespace util;

namespace lattice_planner
{
class LatticePlanner
{
public:
    /**
     * @brief Construct a new Lattice Planner object
     */
    LatticePlanner() = default;

    /**
     * @brief Construct a new Lattice Planner object
     */
    LatticePlanner(std::shared_ptr<CollisionChecker>& collision_checker);

    /**
     * @brief Destroy the Lattice Planner object
     */
    ~LatticePlanner() = default;

    /**
     * @brief plan and generate path
     *
     * @param ref_path reference path
     * @param pose vehicle current pose
     * @param path save generated path
     *
     * @return bool_t true if plan successfully, false otherwise
     */
    virtual bool Plan(const std::vector<CurvePoint>& ref_path, const CurvePoint& pose, const double max_width,
                      const double vehicle_width, const double lattice_length,
                      const std::vector<Point>& obstacle_points, std::vector<CurvePoint>& final_path);

    /**
     * @brief plan and generate path
     *
     * @param ref_path reference path
     * @param pose vehicle current pose
     * @param path save generated path
     *
     * @return bool_t true if plan successfully, false otherwise
     */
    virtual bool Plan(const std::vector<CurvePoint>& ref_path, const geometry_msgs::msg::PoseStamped& pose,
                      const double max_width, const double vehicle_width, const double lattice_length,
                      std::vector<CurvePoint>& final_path);

private:
    bool SelectOptimalPath(const std::vector<std::vector<CurvePoint>>& path_set,
                           const std::vector<Point>& obstacle_points, const double& vehicle_width,
                           std::vector<CurvePoint>& final_path);

    bool SelectOptimalPath(const std::vector<std::vector<CurvePoint>>& path_set, std::vector<CurvePoint>& final_path);

    void ComputeInitFrenetState(const CurvePoint& matched_point, const CurvePoint& pose, std::array<double, 3>* ptr_s,
                                std::array<double, 3>* ptr_d);
    std::vector<CurvePoint> SmoothReferencePath(const std::vector<CurvePoint>& refer_path,
                                                const CurvePoint&              current_pose);

private:
    std::shared_ptr<CollisionChecker> collision_checker_;
};
}  // namespace lattice_planner

#endif  //_LATTICE_PLANNER_H_
