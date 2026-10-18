#include "pure_pursuit.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace pp
{
namespace
{
const double kEpsilon = 1.0e-9;
const double kPi = 3.14159265358979323846;

double clamp(double value, double low, double high)
{
    return std::max(low, std::min(value, high));
}

double length(const Point& point)
{
    return std::hypot(point.x, point.y);
}

double distance(const Point& first, const Point& second)
{
    return std::hypot(first.x - second.x, first.y - second.y);
}

double angleError(double target, double current)
{
    double error = std::fmod(target - current, 360.0);
    if (error > 180.0) error -= 360.0;
    if (error < -180.0) error += 360.0;
    return error;
}

double smoothstep(double value)
{
    const double t = clamp(value, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
}
}

PurePursuit::PurePursuit(const Config& config) : config_(config)
{
    if (config_.lookahead <= 0.0 || config_.min_lookahead <= 0.0 ||
        config_.max_lookahead < config_.min_lookahead ||
        config_.max_speed < 0.0 || config_.position_tolerance < 0.0)
        throw std::invalid_argument("Invalid pure pursuit configuration");
}

void PurePursuit::setPath(const std::vector<Point>& path)
{
    path_.clear();
    cumulative_length_.clear();
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path_.empty() || distance(path_.back(), path[i]) > kEpsilon)
            path_.push_back(path[i]);
    }

    if (!path_.empty())
    {
        cumulative_length_.push_back(0.0);
        for (std::size_t i = 1; i < path_.size(); ++i)
            cumulative_length_.push_back(cumulative_length_.back() +
                                         distance(path_[i - 1], path_[i]));
    }
    clearHeadingTurnRange();
    reset();
}

bool PurePursuit::setHeadingTurnRange(std::size_t start_index,
                                      std::size_t end_index)
{
    if (!hasValidPath() || start_index >= end_index ||
        end_index >= path_.size())
        return false;
    turn_start_ = start_index;
    turn_end_ = end_index;
    turn_range_set_ = true;
    return true;
}

void PurePursuit::clearHeadingTurnRange()
{
    turn_range_set_ = false;
    turn_start_ = 0;
    turn_end_ = 0;
}

void PurePursuit::reset()
{
    last_segment_ = 0;
    last_progress_ = 0.0;
    initial_heading_set_ = false;
    initial_heading_deg_ = 0.0;
}

bool PurePursuit::hasValidPath() const
{
    return path_.size() >= 2;
}

const std::vector<Point>& PurePursuit::path() const
{
    return path_;
}

std::size_t PurePursuit::nearestSegment(const Point& position) const
{
    const std::size_t last = path_.size() - 2;
    const std::size_t end = std::min(last, last_segment_ +
                                    config_.nearest_search_segments);
    std::size_t best = last_segment_;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = last_segment_; i <= end; ++i)
    {
        const Point a = path_[i];
        const Point b = path_[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double ratio = clamp(((position.x - a.x) * dx +
                                    (position.y - a.y) * dy) /
                                   (dx * dx + dy * dy), 0.0, 1.0);
        const Point projection(a.x + ratio * dx, a.y + ratio * dy);
        const double error = distance(position, projection);
        if (error < best_distance)
        {
            best_distance = error;
            best = i;
        }
    }
    return best;
}

double PurePursuit::progress(const Point& position, std::size_t segment) const
{
    const Point a = path_[segment];
    const Point b = path_[segment + 1];
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double ratio = clamp(((position.x - a.x) * dx +
                                (position.y - a.y) * dy) /
                               (dx * dx + dy * dy), 0.0, 1.0);
    return cumulative_length_[segment] +
           ratio * (cumulative_length_[segment + 1] -
                    cumulative_length_[segment]);
}

Point PurePursuit::lookahead(const Point& position, std::size_t nearest,
                             double radius, std::size_t& segment) const
{
    const std::size_t last = path_.size() - 2;
    const std::size_t end = std::min(last, nearest +
                                    config_.lookahead_search_segments);
    bool found = false;
    double best_progress = -1.0;
    Point result = path_.back();

    for (std::size_t i = nearest; i <= end; ++i)
    {
        const Point a = path_[i];
        const Point b = path_[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double fx = a.x - position.x;
        const double fy = a.y - position.y;
        const double quadratic_a = dx * dx + dy * dy;
        const double quadratic_b = 2.0 * (fx * dx + fy * dy);
        const double quadratic_c = fx * fx + fy * fy - radius * radius;
        const double discriminant = quadratic_b * quadratic_b -
                                    4.0 * quadratic_a * quadratic_c;
        if (discriminant < 0.0) continue;
        const double root = std::sqrt(std::max(0.0, discriminant));
        const double roots[2] = {
            (-quadratic_b - root) / (2.0 * quadratic_a),
            (-quadratic_b + root) / (2.0 * quadratic_a)
        };
        for (int j = 0; j < 2; ++j)
        {
            const double t = roots[j];
            if (t < 0.0 || t > 1.0) continue;
            const double arc = cumulative_length_[i] +
                               t * (cumulative_length_[i + 1] -
                                    cumulative_length_[i]);
            if (arc < last_progress_ - kEpsilon || arc <= best_progress)
                continue;
            result = Point(a.x + t * dx, a.y + t * dy);
            segment = i;
            best_progress = arc;
            found = true;
        }
    }
    if (found) return result;

    // 无交点时，从最近投影点沿路径弧长前进一个前视距离。
    double target = progress(position, nearest) + radius;
    for (std::size_t i = nearest; i <= last; ++i)
    {
        if (target > cumulative_length_[i + 1]) continue;
        const double t = clamp((target - cumulative_length_[i]) /
                               (cumulative_length_[i + 1] -
                                cumulative_length_[i]), 0.0, 1.0);
        segment = i;
        return Point(path_[i].x + t * (path_[i + 1].x - path_[i].x),
                     path_[i].y + t * (path_[i + 1].y - path_[i].y));
    }
    segment = last;
    return path_.back();
}

double PurePursuit::curvature(std::size_t segment) const
{
    if (path_.size() < 3) return 0.0;
    const std::size_t middle = std::min(segment + 1, path_.size() - 2);
    const Point a = path_[middle - 1];
    const Point b = path_[middle];
    const Point c = path_[middle + 1];
    const double cross = (b.x - a.x) * (c.y - a.y) -
                         (b.y - a.y) * (c.x - a.x);
    const double denominator = distance(a, b) * distance(b, c) *
                               distance(a, c);
    return denominator > kEpsilon ? 2.0 * cross / denominator : 0.0;
}

Command PurePursuit::update(const Pose& pose, const Point& actual_velocity,
                            double target_heading_deg)
{
    if (!hasValidPath())
        throw std::logic_error("Set a valid path before update");

    Command command;
    if (!initial_heading_set_)
    {
        initial_heading_deg_ = pose.heading_deg;
        initial_heading_set_ = true;
    }

    last_segment_ = nearestSegment(pose.position);
    last_progress_ = std::max(last_progress_,
                              progress(pose.position, last_segment_));
    command.nearest_segment = last_segment_;
    command.remaining_distance = std::max(0.0,
        cumulative_length_.back() - last_progress_);
    command.distance_to_goal = distance(pose.position, path_.back());

    const double actual_speed = length(actual_velocity);
    const double radius = clamp(config_.lookahead +
                                config_.lookahead_speed_gain * actual_speed,
                                config_.min_lookahead, config_.max_lookahead);
    std::size_t lookahead_segment = last_segment_;
    command.lookahead_point = lookahead(pose.position, last_segment_,
                                        radius, lookahead_segment);
    command.lookahead_segment = lookahead_segment;
    command.curvature = curvature(lookahead_segment);

    double heading_ratio = 0.0;
    if (turn_range_set_)
    {
        const double start = cumulative_length_[turn_start_];
        const double end = cumulative_length_[turn_end_];
        heading_ratio = (last_progress_ - start) / (end - start);
    }
    else if (config_.heading_blend_distance > kEpsilon)
    {
        const double start = std::max(0.0, cumulative_length_.back() -
                                      config_.heading_blend_distance);
        const double span = cumulative_length_.back() - start;
        heading_ratio = span > kEpsilon ?
            (last_progress_ - start) / span : 1.0;
    }
    else
    {
        heading_ratio = command.remaining_distance <= kEpsilon ? 1.0 : 0.0;
    }
    command.desired_heading_deg = initial_heading_deg_ +
        smoothstep(heading_ratio) *
        angleError(target_heading_deg, initial_heading_deg_);
    const double heading_error = angleError(command.desired_heading_deg,
                                            pose.heading_deg);
    command.angular_speed_deg_s = clamp(config_.heading_kp * heading_error,
        -config_.max_angular_speed_deg_s,
         config_.max_angular_speed_deg_s);

    command.position_reached =
        command.distance_to_goal <= config_.position_tolerance;
    if (command.position_reached)
    {
        command.velocity = Point();
        command.finished = actual_speed <= config_.stop_speed_tolerance &&
            std::fabs(heading_error) <= config_.heading_tolerance_deg;
        if (command.finished) command.angular_speed_deg_s = 0.0;
        return command;
    }

    const double curve_limit = std::max(0.0,
        std::min(config_.max_speed,
            std::max(config_.min_curve_speed,
                config_.max_speed - config_.curvature_speed_gain *
                std::fabs(command.curvature))));
    double speed = curve_limit;
    if (config_.slowdown_distance > config_.position_tolerance &&
        command.distance_to_goal < config_.slowdown_distance)
    {
        const double ratio = (command.distance_to_goal -
                              config_.position_tolerance) /
                             (config_.slowdown_distance -
                              config_.position_tolerance);
        speed = std::min(speed, config_.max_speed * smoothstep(ratio));
    }

    Point direction(command.lookahead_point.x - pose.position.x,
                    command.lookahead_point.y - pose.position.y);
    const Point to_goal(path_.back().x - pose.position.x,
                        path_.back().y - pose.position.y);
    const double direction_length = length(direction);
    if (direction_length > kEpsilon)
    {
        direction.x /= direction_length;
        direction.y /= direction_length;
    }
    else
    {
        direction = Point(to_goal.x / command.distance_to_goal,
                          to_goal.y / command.distance_to_goal);
    }
    if (config_.terminal_slow_distance > kEpsilon &&
        command.distance_to_goal < config_.terminal_slow_distance)
    {
        const double blend = smoothstep(1.0 - command.distance_to_goal /
                                        config_.terminal_slow_distance);
        direction.x = (1.0 - blend) * direction.x +
                      blend * to_goal.x / command.distance_to_goal;
        direction.y = (1.0 - blend) * direction.y +
                      blend * to_goal.y / command.distance_to_goal;
    }
    const double norm = length(direction);
    if (norm > kEpsilon)
        command.velocity = Point(speed * direction.x / norm,
                                 speed * direction.y / norm);
    return command;
}
}
