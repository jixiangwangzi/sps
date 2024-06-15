#ifndef _CURVE_1D_GENERATOR_H_
#define _CURVE_1D_GENERATOR_H_

#include "nav2_regulated_pure_pursuit_controller/math/curve_1d.h"
#include "nav2_regulated_pure_pursuit_controller/util/collision_checker.hpp"

using namespace util;

namespace lattice_planner
{
/**
 * @brief Class for generate curve set for longitudinal and lateral respectively
 */
class Curve1dGenerator
{
public:
    using Curve1dSet = std::vector<std::shared_ptr<math::Curve1d>>;
    using State      = std::array<double, 3>;
    using Condition  = std::pair<State, double>;
    /**
     * @brief Construct a new Curve 1d Generator object
     *
     * @param init_lon_state initial longitudinal frenet state
     * @param init_lat_state initial lateral frenet state
     * @param obstacle_points obstacle points
     */
    /*explicit */ Curve1dGenerator(const CurvePoint& start_pose, const CurvePoint& start_match_point,
                                   const State& init_lon_state, const State& init_lat_state,
                                   const std::vector<CurvePoint>& refer_path,
                                   const std::vector<Point>&      obstacle_points);
    Curve1dGenerator(const CurvePoint& start_pose, const CurvePoint& start_match_point, const State& init_lon_state,
                     const State& init_lat_state, const std::vector<CurvePoint>& refer_path,
                     std::shared_ptr<CollisionChecker>& collision_checker);

    /**
     * @brief Destroy the Curve 1d Generator object
     */
    ~Curve1dGenerator() = default;

    /**
     * @brief generate longitudinal and lateral curve sets
     *
     * @param left_road_width left road width
     * @param right_road_width right road width
     * @param vehicle_width vehicle width
     * @param lat_curve_set set to save lateral curves
     */
    void GenerateCurveSet(double left_road_width, double right_road_width, double vehicle_width, double lattice_length,
                          Curve1dSet& lat_curve_set);

private:
    /**
     * @brief sample lateral end conditions
     *
     * @param lat_curve_set set to save lateral curves
     * @param left_road_width right road width
     * @param right_road_width road width
     * @param vehicle_width ego car width
     */
    void SampleLateralEndConditions(double left_road_width, double right_road_width, double vehicle_width,
                                    double target_s, std::vector<Condition>& LatEndConditions);
    /**
     * @brief generate lateral curve set
     *
     * @param lat_curve_set set to save lateral curves
     * @param left_road_width right road width
     * @param right_road_width road width
     * @param vehicle_width ego car width
     */
    void GenerateLatCurveSet(Curve1dSet& lat_curve_set, double left_road_width, double right_road_width,
                             double vehicle_width, double lattice_length);

    /**
     * @brief transform frenet to cartesian
     *
     * @param s_condition
     * @param l_condition
     * @return the cartesian coordinate
     */
    Point TransformToCartesian(std::array<double, 3> s_condition, std::array<double, 3> l_condition);

    /**
     * @brief transform start pose to frenet cordinate (head from start to target)
     *
     * @param target_point
     * @return the frenet coordinate
     */
    State TransformStartPoseStateByTargetPoint(const Point& target_point);

    /**
     * @brief initial pose
     */
    CurvePoint start_pose_;

    /**
     * @brief initial pose
     */
    CurvePoint start_match_point_;

    /**
     * @brief initial longitudinal frenet state
     */
    State init_lon_state_;

    /**
     * @brief initial lateral frenet state
     */
    State init_lat_state_;

    /**
     * @brief reference path
     */
    std::vector<CurvePoint> refer_path_;

    /**
     * @brief obstacle points
     */
    std::vector<Point> obstacle_points_;

    /**
     * @brief collision checker
     */
    std::shared_ptr<CollisionChecker> collision_checker_;
};

}  // namespace lattice_planner

#endif
