#ifndef _LATTICE_CURVE_EVALUATOR_H_
#define _LATTICE_CURVE_EVALUATOR_H_

#include <queue>

#include "nav2_regulated_pure_pursuit_controller/lattice_planner/lattice_planner.h"
#include "nav2_regulated_pure_pursuit_controller/math/curve_1d.h"

namespace lattice_planner
{
/**
 * @brief Class for generate curve set for longitudinal and lateral respectively
 */
class CurveEvaluator
{
public:
    using State                  = std::array<double, 3>;
    using Curve1dSet             = std::vector<std::shared_ptr<math::Curve1d>>;
    using PtrCurve1d             = std::shared_ptr<math::Curve1d>;
    using PairWithTotalCost      = std::pair<Curve1dSet, double>;
    using PairWithRespectiveCost = std::pair<PtrCurve1d, double>;

    /**
     * @brief Construct the Curve Evaluator object
     */
    CurveEvaluator() = default;

    /**
     * @brief Destroy the Curve Evaluator object
     */
    ~CurveEvaluator() = default;

    /**
     * @brief lateral curve, evaluate all the pairs and sort them
     *
     * @param lat_curve_set lateral curves set
     */
    void Evaluate(const Curve1dSet& lat_curve_set);

    /**
     * @brief judge whether sorted queue of saving pairs has pair
     *
     * @return bool true if queue has pair, false if queue is empty
     */
    inline bool HasTrajectoryPairs() const
    {
        return !cost_queue_.empty();
    }

    /**
     * @brief return the size of the queue
     *
     * @return size_t the size of the queue
     */
    inline size_t GetSize() const
    {
        return cost_queue_.size();
    }

    /**
     * @brief get the first pair of the queue and then pop it
     *
     * @return PtrCurve1dPair the first pair of the queue
     */
    PtrCurve1d PopBestTrajectory()
    {
        if (cost_queue_.empty())
        {
            std::cout << "trajectory queue empty" << std::endl;
            PtrCurve1d empty_curve(nullptr);
            return empty_curve;
        }
        PairWithRespectiveCost top = cost_queue_.top();
        cost_queue_.pop();
        return top.first;
    }

    /**
     * @brief get the first pair's cost of the queue
     *
     * @return const std::pair<double, double>& the reference of cost pair of the first curves pair
     */
    inline const double& GetBestTrajectoryPairCost() const
    {
        return cost_queue_.top().second;
    }

private:
    /**
     * @brief calculate curve lateral cost
     *
     * @param ptr_lat_curve pointer of the lateral curve
     *
     * @return double the cost
     */
    double CalLatCost(const std::shared_ptr<math::Curve1d>& ptr_lat_curve) const;

    /**
     * @brief struct of cost comparator
     */
    struct CostComparator
      : public std::binary_function<const PairWithRespectiveCost&, const PairWithRespectiveCost&, bool>
    {
        bool operator()(const PairWithRespectiveCost& left, const PairWithRespectiveCost& right) const
        {
            return left.second > right.second;
        }
    };

    /**
     * @brief queue to save curves pair and its cost
     */
    std::priority_queue<PairWithRespectiveCost, std::vector<PairWithRespectiveCost>, CostComparator> cost_queue_;
};

}  // namespace lattice_planner

#endif
