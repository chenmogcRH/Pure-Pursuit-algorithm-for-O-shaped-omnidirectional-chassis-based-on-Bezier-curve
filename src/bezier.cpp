#include "bezier.h"
#include <stdexcept>

namespace pp
{
Point bezierPoint(const std::vector<Point>& control_points, double t)
{
    if (control_points.empty() || t < 0.0 || t > 1.0)
        throw std::invalid_argument("Bezier control points or t are invalid");

    std::vector<Point> work = control_points;
    for (std::size_t level = work.size() - 1; level > 0; --level)
    {
        for (std::size_t i = 0; i < level; ++i)
        {
            work[i].x = (1.0 - t) * work[i].x + t * work[i + 1].x;
            work[i].y = (1.0 - t) * work[i].y + t * work[i + 1].y;
        }
    }
    return work.front();
}

std::vector<Point> sampleBezier(const std::vector<Point>& control_points,
                                unsigned int segments)
{
    if (control_points.size() < 2 || segments == 0)
        throw std::invalid_argument("Bezier needs two points and one segment");

    std::vector<Point> path;
    path.reserve(static_cast<std::size_t>(segments) + 1);
    for (unsigned int i = 0; i <= segments; ++i)
        path.push_back(bezierPoint(control_points,
                                   static_cast<double>(i) / segments));
    return path;
}
}

