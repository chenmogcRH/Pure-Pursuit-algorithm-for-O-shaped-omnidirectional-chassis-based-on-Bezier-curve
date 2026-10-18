#ifndef PP_BEZIER_H
#define PP_BEZIER_H

#include "geometry.h"
#include <vector>

namespace pp
{
// n 阶贝塞尔曲线；样本数为段数，返回样本数 + 1 个点。
Point bezierPoint(const std::vector<Point>& control_points, double t);
std::vector<Point> sampleBezier(const std::vector<Point>& control_points,
                                unsigned int segments);
}

#endif

