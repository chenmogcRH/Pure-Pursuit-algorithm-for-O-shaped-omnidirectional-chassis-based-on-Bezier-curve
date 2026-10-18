#ifndef PP_PURE_PURSUIT_H
#define PP_PURE_PURSUIT_H

#include "geometry.h"
#include <cstddef>
#include <vector>

namespace pp
{
struct Config
{
    double lookahead = 25.0;
    double lookahead_speed_gain = 0.20;
    double min_lookahead = 22.0;
    double max_lookahead = 40.0;
    double max_speed = 50.0;
    double min_curve_speed = 35.0;
    double curvature_speed_gain = 1500.0;
    double slowdown_distance = 60.0;
    double terminal_slow_distance = 30.0;
    double position_tolerance = 2.0;
    double stop_speed_tolerance = 5.0;
    double heading_tolerance_deg = 2.0;
    double heading_blend_distance = 100.0;
    double heading_kp = 1.0;
    double max_angular_speed_deg_s = 90.0;
    std::size_t nearest_search_segments = 80;
    std::size_t lookahead_search_segments = 80;
};

struct Command
{
    Point velocity;                 // 全局坐标中的速度，长度单位/秒
    double angular_speed_deg_s = 0.0;
    Point lookahead_point;
    std::size_t nearest_segment = 0;
    std::size_t lookahead_segment = 0;
    double remaining_distance = 0.0;
    double distance_to_goal = 0.0;
    double curvature = 0.0;
    double desired_heading_deg = 0.0;
    bool position_reached = false;
    bool finished = false;
};

class PurePursuit
{
public:
    explicit PurePursuit(const Config& config = Config());
    void setPath(const std::vector<Point>& path);
    bool setHeadingTurnRange(std::size_t start_index, std::size_t end_index);
    void clearHeadingTurnRange();
    void reset();
    bool hasValidPath() const;
    const std::vector<Point>& path() const;

    // pose、实际全局速度和目标终点角度均由调用方提供。
    // 角度约定：全局 +X 为 0°，逆时针为正。
    Command update(const Pose& pose, const Point& actual_velocity,
                   double target_heading_deg);

private:
    Config config_;
    std::vector<Point> path_;
    std::vector<double> cumulative_length_;
    std::size_t last_segment_ = 0;
    double last_progress_ = 0.0;
    bool initial_heading_set_ = false;
    double initial_heading_deg_ = 0.0;
    bool turn_range_set_ = false;
    std::size_t turn_start_ = 0;
    std::size_t turn_end_ = 0;

    std::size_t nearestSegment(const Point& position) const;
    Point lookahead(const Point& position, std::size_t nearest,
                    double radius, std::size_t& segment) const;
    double progress(const Point& position, std::size_t segment) const;
    double curvature(std::size_t segment) const;
};
}

#endif
