#include "dubins.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dubins {
namespace {
bool intersects(const Box &b, double x0, double y0, double z0, double x1, double y1, double z1,
                double m) {
    return x1 >= b.x0 - m && x0 <= b.x1 + m && y1 >= b.y0 - m && y0 <= b.y1 + m && z1 >= b.z0 - m &&
           z0 <= b.z1 + m;
}
// A collision check evaluates many points on the SAME curve. Cache the
// full segment prefixes and their sin/cos once, retaining the original
// operation order (including sequential distance subtraction).
struct Curve {
    const Path &path;
    struct Frame {
        double x, y, yaw, sine, cosine;
    };
    std::array<Frame, 4> frames{};

    explicit Curve(const Path &p) : path(p) {
        double x = p.start.x, y = p.start.y, yaw = p.start.yaw;
        for (int i = 0; i < p.count; ++i) {
            auto &frame = frames[i];
            frame = {x, y, yaw, std::sin(yaw), std::cos(yaw)};
            const auto &segment = p.segments[i];
            if (segment.kind == 2) {
                x += segment.length * frame.cosine;
                y += segment.length * frame.sine;
            } else {
                double sign = segment.kind == 0 ? 1 : -1;
                double next_yaw = yaw + sign * segment.length / p.radius;
                x += sign * p.radius * (std::sin(next_yaw) - frame.sine);
                y += sign * p.radius * (frame.cosine - std::cos(next_yaw));
                yaw = next_yaw;
            }
        }
    }
    Pose at(double distance) const {
        if (path.horizontal == 0)
            return path.start;
        double remaining = distance;
        Pose p = path.start;
        for (int i = 0; i < path.count; ++i) {
            const auto &segment = path.segments[i];
            if (remaining > segment.length && i + 1 < path.count) {
                remaining -= segment.length;
                continue;
            }
            const auto &frame = frames[i];
            double d = std::min(remaining, segment.length);
            p.x = frame.x;
            p.y = frame.y;
            p.yaw = frame.yaw;
            if (segment.kind == 2) {
                p.x += d * frame.cosine;
                p.y += d * frame.sine;
            } else {
                double sign = segment.kind == 0 ? 1 : -1;
                p.yaw += sign * d / path.radius;
                p.x += sign * path.radius * (std::sin(p.yaw) - frame.sine);
                p.y += sign * path.radius * (frame.cosine - std::cos(p.yaw));
            }
            break;
        }
        p.z = path.start.z + (path.goal.z - path.start.z) * distance / path.horizontal;
        // World::free only tests yaw finiteness, not its wrapped value.
        return p;
    }
};
// Identical interval bounds and subdivision tolerance to the Python checker.
// Every intersected raster cell is checked: thin obstacles cannot fall
// between samples. No cell lists or Pose objects are allocated here.
bool interval(const World &w, const Curve &curve, double a, double b, double minimum) {
    const auto &p = curve.path;
    Pose midpoint = curve.at((a + b) / 2);
    if (!w.free(midpoint))
        return false;
    double half = (b - a) / 2;
    // Only altitude is needed at the interval ends. Evaluate the SAME
    // linear-height expression without two redundant XY/trigonometry walks.
    double za = p.horizontal ? p.start.z + (p.goal.z - p.start.z) * a / p.horizontal : p.start.z;
    double zb = p.horizontal ? p.start.z + (p.goal.z - p.start.z) * b / p.horizontal : p.start.z;
    double z0 = std::min(za, zb), z1 = std::max(za, zb);
    double x0 = midpoint.x - half, y0 = midpoint.y - half, x1 = midpoint.x + half,
           y1 = midpoint.y + half;
    int r0, c0, r1, c1;
    bool clear = w.cell(x0, y0, r0, c0) && w.cell(x1, y1, r1, c1);
    if (clear) {
        for (int r = r0; r <= r1 && clear; ++r)
            for (int c = c0; c <= c1; ++c) {
                int index = r * w.columns + c;
                if (!(w.lower[index] < z0 && z1 < w.upper[index])) {
                    clear = false;
                    break;
                }
            }
    }
    if (clear)
        for (int i = 0; i < w.boxes_count; ++i)
            if (intersects(w.boxes[i], x0, y0, z0, x1, y1, z1, w.margin)) {
                clear = false;
                break;
            }
    if (clear)
        return true;
    if (b - a <= minimum)
        return false;
    double middle = (a + b) / 2;
    return interval(w, curve, a, middle, minimum) && interval(w, curve, middle, b, minimum);
}
} // namespace
bool World::cell(double x, double y, int &r, int &c) const {
    if (!std::isfinite(x) || !std::isfinite(y) || x < x0 || y < y0 ||
        x >= x0 + columns * resolution || y >= y0 + rows * resolution)
        return false;
    r = static_cast<int>(std::floor((y - y0) / resolution));
    c = static_cast<int>(std::floor((x - x0) / resolution));
    // Subtraction/division can round an interior boundary to rows/columns.
    return r >= 0 && r < rows && c >= 0 && c < columns;
}
bool World::free(const Pose &p) const {
    int r, c;
    if (!std::isfinite(p.z) || !std::isfinite(p.yaw) || !cell(p.x, p.y, r, c))
        return false;
    int index = r * columns + c;
    if (!(lower[index] < p.z && p.z < upper[index]))
        return false;
    for (int i = 0; i < boxes_count; ++i)
        if (intersects(boxes[i], p.x, p.y, p.z, p.x, p.y, p.z, margin))
            return false;
    return true;
}
bool World::path_free(const Path &p, double step, double minimum) const {
    if (!std::isfinite(step) || !std::isfinite(minimum) || minimum <= 0 || step < minimum)
        throw std::invalid_argument("require 0 < min_step <= max_step");
    if (!free(p.start) || !free(p.goal))
        return false;
    Curve curve(p);
    double n = std::max(1.0, std::ceil(p.horizontal / step));
    for (double i = 0; i < n; ++i)
        if (!interval(*this, curve, std::min(p.horizontal, p.horizontal * i / n),
                      std::min(p.horizontal, p.horizontal * (i + 1) / n), minimum))
            return false;
    return true;
}
} // namespace dubins
