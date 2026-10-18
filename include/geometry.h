#ifndef PP_GEOMETRY_H
#define PP_GEOMETRY_H

namespace pp
{
struct Point
{
    double x;
    double y;

    Point(double x_value = 0.0, double y_value = 0.0)
        : x(x_value), y(y_value) {}
};

struct Pose
{
    Point position;
    double heading_deg;

    Pose(Point point = Point(), double heading = 0.0)
        : position(point), heading_deg(heading) {}
};
}

#endif

