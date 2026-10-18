#include "bezier.h"
#include "pure_pursuit.h"
#include <cassert>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace
{
bool near(double a, double b, double tolerance = 1.0e-6)
{
    return std::fabs(a - b) <= tolerance;
}
}

int main()
{
    const std::vector<pp::Point> controls = {
        pp::Point(0.0, 0.0), pp::Point(50.0, 100.0),
        pp::Point(100.0, 0.0)
    };
    const std::vector<pp::Point> curve = pp::sampleBezier(controls, 10);
    assert(curve.size() == 11);
    assert(near(curve.front().x, 0.0));
    assert(near(curve.back().x, 100.0));
    assert(near(curve[5].x, 50.0));
    assert(near(curve[5].y, 50.0));

    bool invalid_bezier = false;
    try { pp::sampleBezier(controls, 0); }
    catch (const std::invalid_argument&) { invalid_bezier = true; }
    assert(invalid_bezier);

    pp::PurePursuit tracker;
    tracker.setPath({pp::Point(0.0, 0.0), pp::Point(0.0, 0.0),
                     pp::Point(100.0, 0.0)});
    assert(tracker.path().size() == 2);
    assert(!tracker.setHeadingTurnRange(1, 1));
    assert(tracker.setHeadingTurnRange(0, 1));

    const pp::Command start = tracker.update(pp::Pose(pp::Point(0.0, 0.0), 0.0),
                                              pp::Point(), 90.0);
    assert(near(start.lookahead_point.x, 25.0));
    assert(near(start.lookahead_point.y, 0.0));
    assert(near(start.velocity.x, 50.0));
    assert(near(start.velocity.y, 0.0));
    assert(near(start.desired_heading_deg, 0.0));

    const pp::Command middle = tracker.update(pp::Pose(pp::Point(50.0, 0.0), 0.0),
                                               pp::Point(), 90.0);
    assert(near(middle.desired_heading_deg, 45.0));
    assert(middle.velocity.x < start.velocity.x);
    assert(middle.remaining_distance < start.remaining_distance);

    const pp::Command end = tracker.update(pp::Pose(pp::Point(100.0, 0.0), 90.0),
                                            pp::Point(), 90.0);
    assert(end.position_reached && end.finished);
    assert(near(end.velocity.x, 0.0));
    assert(near(end.angular_speed_deg_s, 0.0));

    tracker.reset();
    const pp::Command moving_end = tracker.update(
        pp::Pose(pp::Point(100.0, 0.0), 90.0), pp::Point(10.0, 0.0), 90.0);
    assert(moving_end.position_reached && !moving_end.finished);
    return 0;
}
