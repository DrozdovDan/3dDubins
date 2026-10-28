#include "dubins.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

namespace dubins {
namespace {
double wrap(double value) {
    value = std::fmod(value, tau);
    if (value < 0)
        value += tau;
    return tau - value < 1e-12 ? 0 : value;
}
double angle_error(double a, double b) {
    double value = std::fmod(a - b + pi, tau);
    if (value < 0)
        value += tau;
    return value - pi;
}
struct CSC {
    int word;
    std::array<double, 3> lengths;
    double total() const { return lengths[0] + lengths[1] + lengths[2]; }
};
constexpr const char *words[] = {"RSR", "RSL", "LSR", "LSL"};
struct CSCProblem {
    double distance, alpha, beta, sa, sb, ca, cb, cab, start_yaw, goal_yaw;
    CSCProblem(const Pose &a, const Pose &b) : start_yaw(a.yaw), goal_yaw(b.yaw) {
        double dx = b.x - a.x, dy = b.y - a.y;
        distance = std::hypot(dx, dy);
        double theta = std::atan2(dy, dx);
        alpha = wrap(a.yaw - theta);
        beta = wrap(b.yaw - theta);
        sa = std::sin(alpha);
        sb = std::sin(beta);
        ca = std::cos(alpha);
        cb = std::cos(beta);
        cab = std::cos(alpha - beta);
    }
    CSC at(double radius) const {
        if (distance < 1e-12 * radius && std::abs(angle_error(start_yaw, goal_yaw)) < 1e-12)
            return {0, {0, 0, 0}};
        double d = distance / radius;
        CSC best{0, {std::numeric_limits<double>::infinity(), 0, 0}};
        auto accept = [&](int word, double square, auto angles) {
            if (square < -1e-12)
                return;
            double p = std::sqrt(std::max(0.0, square));
            auto pair = angles(p);
            CSC candidate{word,
                          {wrap(pair.first) * radius, p * radius, wrap(pair.second) * radius}};
            if (candidate.total() < best.total())
                best = candidate;
        };
        // Match the Python/reference tie-breaking order, not formula order.
        accept(0, 2 + d * d - 2 * cab + 2 * d * (sb - sa), [&](double) {
            double t = std::atan2(ca - cb, d - sa + sb);
            return std::pair<double, double>{alpha - t, -beta + t};
        });
        accept(1, -2 + d * d + 2 * cab - 2 * d * (sa + sb), [&](double p) {
            double t = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
            return std::pair<double, double>{alpha - t, beta - t};
        });
        accept(2, -2 + d * d + 2 * cab + 2 * d * (sa + sb), [&](double p) {
            double t = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
            return std::pair<double, double>{-alpha + t, -beta + t};
        });
        accept(3, 2 + d * d - 2 * cab + 2 * d * (sa - sb), [&](double) {
            double t = std::atan2(cb - ca, d + sa - sb);
            return std::pair<double, double>{-alpha + t, beta - t};
        });
        return best;
    }
};
CSC shortest(const Pose &a, const Pose &b, double radius) { return CSCProblem(a, b).at(radius); }
Pose turn(const Pose &p, char kind, double radius, double angle) {
    double direction = kind == 'L' ? 1 : -1, yaw = p.yaw + direction * angle;
    return {p.x + direction * radius * (std::sin(yaw) - std::sin(p.yaw)),
            p.y + direction * radius * (std::cos(p.yaw) - std::cos(yaw)), p.z, wrap(yaw)};
}
template <class Function>
std::optional<double> bisect(Function &f, double low, double high, double tolerance, double fl,
                             double fh) {
    if (!std::isfinite(fl) || !std::isfinite(fh))
        return {};
    if (std::abs(fl) <= tolerance)
        return low;
    if (std::abs(fh) <= tolerance)
        return high;
    if (fl * fh > 0)
        return {};
    for (int i = 0; i < 100; ++i) {
        double middle = (low + high) / 2, value = f(middle);
        if (!std::isfinite(value))
            return {};
        if (std::abs(value) <= tolerance)
            return middle;
        if (fl * value <= 0)
            high = middle;
        else {
            low = middle;
            fl = value;
        }
    }
    return {}; // Wrapped-arc discontinuities are not roots.
}
template <class Function> double root(Function f, double low, double high, double tolerance) {
    const double fl = f(low), fh = f(high);
    if (auto r = bisect(f, low, high, tolerance, fl, fh))
        return *r;
    double previous = low, previous_value = fl;
    for (int i = 1; i <= 256; ++i) {
        double current = low + (high - low) * i / 256;
        // Adjacent brackets share an endpoint. Reusing its exact value
        // halves expensive CSC evaluations in the fallback scan without
        // changing the brackets, tolerances or root selection order.
        double current_value = f(current);
        if (auto r = bisect(f, previous, current, tolerance, previous_value, current_value))
            return *r;
        previous = current;
        previous_value = current_value;
    }
    throw std::invalid_argument(
        "Owen's selected CSC branch has no continuous length-equation solution. "
        "Nearby poses can require CCC paths; use separated configurations.");
}
bool finite(const Pose &p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) && std::isfinite(p.yaw);
}
} // namespace

Pose Path::pose_at(double distance) const {
    if (!std::isfinite(distance) || distance < 0 || distance > horizontal)
        throw std::invalid_argument("horizontal_distance must lie on the path");
    if (horizontal == 0)
        return start;
    Pose p = start;
    double remaining = distance;
    for (int i = 0; i < count; ++i) {
        double d = std::min(remaining, segments[i].length);
        if (segments[i].kind == 2) {
            p.x += d * std::cos(p.yaw);
            p.y += d * std::sin(p.yaw);
        } else {
            double sign = segments[i].kind == 0 ? 1 : -1;
            double yaw = p.yaw + sign * d / radius;
            p.x += sign * radius * (std::sin(yaw) - std::sin(p.yaw));
            p.y += sign * radius * (std::cos(p.yaw) - std::cos(yaw));
            p.yaw = yaw;
        }
        remaining -= d;
        if (remaining <= 0)
            break;
    }
    p.z = start.z + (goal.z - start.z) * distance / horizontal;
    p.yaw = wrap(p.yaw);
    return p;
}

Path plan(const Pose &start, const Pose &goal, double radius, double max_angle) {
    if (!finite(start) || !finite(goal) || !std::isfinite(radius) || !std::isfinite(max_angle) ||
        radius <= 0 || max_angle <= 0 || max_angle >= pi / 2)
        throw std::invalid_argument(
            "finite poses, positive radius and angle in (0, pi/2) required");
    Path path{};
    path.start = start;
    path.goal = goal;
    path.radius = path.minimum_radius = radius;
    path.max_angle = max_angle;
    double dz = goal.z - start.z, required = std::abs(dz) / std::tan(max_angle);
    // Radius fitting solves the SAME endpoint problem dozens of times.
    // Its bearings and trig terms do not depend on the trial radius.
    const CSCProblem problem(start, goal);
    CSC base = problem.at(radius);
    if (!std::isfinite(base.total()) || !std::isfinite(required))
        throw std::invalid_argument("configuration exceeds the numeric range of the planner");
    path.base_word = base.word;
    std::string word = words[base.word];
    std::array<double, 4> lengths{base.lengths[0], base.lengths[1], base.lengths[2], 0};
    int count = 3;
    if (required <= base.total())
        path.altitude_case = 0;
    else if (required < base.total() + tau * radius) {
        path.altitude_case = 1;
        bool climb = dz > 0;
        char kind = climb ? word.front() : word.back();
        auto candidate = [&](double phi) {
            Pose intermediate = turn(climb ? start : goal, kind, radius, climb ? phi : -phi);
            return shortest(climb ? intermediate : start, climb ? goal : intermediate, radius);
        };
        auto residual = [&](double phi) {
            const auto c = candidate(phi);
            const double length =
                climb ? ((radius * phi + c.lengths[0]) + c.lengths[1]) + c.lengths[2]
                      : c.total() + radius * phi;
            return length - required;
        };
        path.extension = root(residual, 0, tau, 1e-10 * std::max(1.0, required));
        CSC inner = candidate(path.extension);
        if (climb) {
            word = std::string(1, kind) + words[inner.word];
            lengths = {radius * path.extension, inner.lengths[0], inner.lengths[1],
                       inner.lengths[2]};
        } else {
            word = std::string(words[inner.word]) + kind;
            lengths = {inner.lengths[0], inner.lengths[1], inner.lengths[2],
                       radius * path.extension};
        }
        count = 4;
    } else {
        path.altitude_case = 2;
        double turns = std::max(1.0, std::floor((required - base.total()) / (tau * radius)));
        if (turns > std::numeric_limits<int>::max())
            throw std::invalid_argument("too many turns");
        path.turns = static_cast<int>(turns);
        auto residual = [&](double r) {
            return problem.at(r).total() + tau * path.turns * r - required;
        };
        path.radius = root(residual, radius, 2 * radius, 1e-10 * std::max(1.0, required));
        CSC fitted = problem.at(path.radius);
        word = words[fitted.word];
        lengths = {fitted.lengths[0], fitted.lengths[1], fitted.lengths[2], 0};
        lengths[dz > 0 ? 0 : 2] += tau * path.turns * path.radius;
    }
    std::copy(word.begin(), word.end(), path.word.begin());
    for (int i = 0; i < count; ++i)
        if (lengths[i] > 1e-12) {
            path.segments[path.count++] = {lengths[i], word[i] == 'L' ? 0 : word[i] == 'R' ? 1 : 2};
            path.horizontal += lengths[i];
        }
    path.gamma = path.horizontal ? std::atan2(dz, path.horizontal) : 0;
    path.length = std::hypot(path.horizontal, dz);
    Pose end = path.pose_at(path.horizontal);
    if (!finite(end) || !std::isfinite(path.length) ||
        std::hypot(end.x - goal.x, end.y - goal.y) > 1e-7 * std::max(1.0, path.radius) ||
        std::abs(angle_error(end.yaw, goal.yaw)) > 1e-8)
        throw std::runtime_error("internal Dubins calculation did not reach the goal");
    return path;
}
} // namespace dubins
