/*!
 *  \brief
 *  \author sunlei (sunlei@holomatic.com)
 *  \date 2018-10-31
 *  \attention Copyright © Holomatic Technology (Beijing) Co.Ltd
 *  \attention Please refer to COPYRIGHT.txt for complete terms of copyright notice.
 */

#ifndef _UTIL_PATH_MATCHER_H_
#define _UTIL_PATH_MATCHER_H_

#include <vector>

#include "nav2_regulated_pure_pursuit_controller/common/curve_point.h"

using namespace common;

namespace util
{
class PathMatcher
{
public:
    PathMatcher() = delete;

    static CurvePoint Match2Path(const std::vector<CurvePoint>& reference_line, const double x, const double y);

    static CurvePoint Match2Path(const std::vector<CurvePoint>& reference_line, const double s);

    static std::pair<double, double> GetPathFrenetCoordinate(const std::vector<CurvePoint>& reference_line,
                                                             const double x, const double y);

private:
    static CurvePoint FindProjectionPoint(const CurvePoint& p0, const CurvePoint& p1, const double x, const double y);
};

}  // namespace util

#endif
