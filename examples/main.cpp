#include "bezier.h"
#include "pure_pursuit.h"
#include <iomanip>
#include <iostream>
#include <vector>

int main()
{
    const std::vector<pp::Point> control_points = {
        pp::Point(0.0, 0.0),
        pp::Point(40.0, 100.0),
        pp::Point(120.0, 100.0),
        pp::Point(160.0, 0.0)
    };

    pp::PurePursuit tracker;
    tracker.setPath(pp::sampleBezier(control_points, 100));
    if (!tracker.setHeadingTurnRange(40, 70)) return 1;

    std::cout << "index,x,y,lookahead_x,lookahead_y,vx,vy,omega,heading\n";
    for (std::size_t i = 0; i < tracker.path().size(); i += 10)
    {
        // 演示输入：这里把路径采样点当作当前位置，并非真实运动仿真。
        const pp::Pose pose(tracker.path()[i], 0.0);
        const pp::Command command = tracker.update(pose, pp::Point(), 90.0);
        std::cout << i << ',' << std::fixed << std::setprecision(2)
                  << pose.position.x << ',' << pose.position.y << ','
                  << command.lookahead_point.x << ','
                  << command.lookahead_point.y << ','
                  << command.velocity.x << ',' << command.velocity.y << ','
                  << command.angular_speed_deg_s << ','
                  << command.desired_heading_deg << '\n';
    }
    return 0;
}

