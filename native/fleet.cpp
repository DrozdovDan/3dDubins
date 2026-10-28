#include "dubins.hpp"
#include "curve.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace dubins {
namespace {
struct Motion {
    const TimedFlight &flight;
    double duration = 0, start_phase, goal_phase;
    int start_direction, goal_direction;
    std::vector<detail::Curve> curves;

    static int direction(const Circle &circle, const Pose &pose) {
        double phase = std::atan2(pose.y - circle.y, pose.x - circle.x);
        return std::cos(pose.yaw - phase - pi / 2) > 0 ? 1 : -1;
    }
    explicit Motion(const TimedFlight &f) : flight(f) {
        if (!f.paths || f.count <= 0 || !std::isfinite(f.speed) || f.speed <= 0 ||
            !std::isfinite(f.departure) || f.departure < 0)
            throw std::invalid_argument("invalid timed flight");
        for (const auto &c : {f.start, f.goal})
            if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z) ||
                !std::isfinite(c.radius) || c.radius <= 0)
                throw std::invalid_argument("invalid timed loiter");
        for (int i = 0; i < f.count; ++i) {
            const auto &p = f.paths[i];
            if (!std::isfinite(p.length) || p.length < 0 || !std::isfinite(p.horizontal) ||
                p.horizontal < 0 || !std::isfinite(p.radius) || p.radius <= 0 || p.count < 0 || p.count > 4)
                throw std::invalid_argument("invalid timed path geometry");
            for (const auto &pose : {p.start, p.goal})
                if (!std::isfinite(pose.x) || !std::isfinite(pose.y) ||
                    !std::isfinite(pose.z) || !std::isfinite(pose.yaw))
                    throw std::invalid_argument("invalid timed path pose");
            for (int j = 0; j < p.count; ++j)
                if (!std::isfinite(p.segments[j].length) || p.segments[j].length < 0 ||
                    p.segments[j].kind < 0 || p.segments[j].kind > 2)
                    throw std::invalid_argument("invalid timed path segment");
            duration += p.length / f.speed;
            curves.emplace_back(p);
        }
        const auto &a = f.paths[0].start, &b = f.paths[f.count - 1].goal;
        start_phase = std::atan2(a.y - f.start.y, a.x - f.start.x);
        goal_phase = std::atan2(b.y - f.goal.y, b.x - f.goal.x);
        start_direction = direction(f.start, a);
        goal_direction = direction(f.goal, b);
    }
    Pose circle(const Circle &c, double phase, int sign, double elapsed) const {
        phase += sign * flight.speed * elapsed / c.radius;
        return {c.x + c.radius * std::cos(phase), c.y + c.radius * std::sin(phase), c.z,
                phase + sign * pi / 2};
    }
    Pose at(double time) const {
        double elapsed = time - flight.departure;
        if (elapsed < 0)
            return circle(flight.start, start_phase, start_direction, elapsed);
        if (elapsed >= duration)
            return circle(flight.goal, goal_phase, goal_direction, elapsed - duration);
        double distance = elapsed * flight.speed;
        for (int i = 0; i < flight.count; ++i) {
            const auto &p = flight.paths[i];
            if (distance <= p.length)
                return p.length ? curves[i].at(std::min(p.horizontal, distance * p.horizontal / p.length))
                                : p.start;
            distance -= p.length;
        }
        return flight.paths[flight.count - 1].goal;
    }
};
struct PathMotion {
    const Path &path;
    double speed, departure;
    detail::Curve curve;
    PathMotion(const Path &p, double s, double t) : path(p), speed(s), departure(t), curve(p) {}
    Pose at(double time) const {
        double d = std::clamp((time - departure) * speed, 0.0, path.length);
        return path.length ? curve.at(std::min(path.horizontal, d * path.horizontal / path.length))
                           : path.start;
    }
    double altitude(double time) const {
        double d = std::clamp((time-departure)*speed, 0.0, path.length);
        double h = path.length ? std::min(path.horizontal, d*path.horizontal/path.length) : 0;
        return path.horizontal ? path.start.z+(path.goal.z-path.start.z)*h/path.horizontal : path.start.z;
    }
};
bool vertically_certified(const PathMotion &own, const Motion &peer, double separation) {
    double begin=own.departure, end=begin+own.path.length/own.speed;
    double low=INFINITY, high=-INFINITY;
    double scale=std::max({1.0,std::abs(own.path.start.z),std::abs(own.path.goal.z),
                           std::abs(peer.flight.start.z),std::abs(peer.flight.goal.z)});
    auto sample = [&](double at, double z) {
        double d=own.altitude(at)-z;
        low=std::min(low,d); high=std::max(high,d);
    };
    auto constant = [&](double left, double right, double z) {
        left=std::max(begin,left); right=std::min(end,right);
        if (left<=right) { sample(left,z); sample(right,z); }
    };
    const auto &f=peer.flight;
    constant(begin,f.departure,f.start.z);
    double prefix=0;
    for (int i=0; i<f.count; ++i) {
        const auto &p=f.paths[i];
        scale=std::max({scale,std::abs(p.start.z),std::abs(p.goal.z)});
        double a=f.departure+prefix/f.speed;
        double b=f.departure+(prefix+p.length)/f.speed;
        double left=std::max(begin,a), right=std::min(end,b);
        if (left<=right) {
            auto z = [&](double at) {
                double d=std::clamp((at-f.departure)*f.speed-prefix,0.0,p.length);
                double h=p.length ? std::min(p.horizontal,d*p.horizontal/p.length) : 0;
                return p.horizontal ? p.start.z+(p.goal.z-p.start.z)*h/p.horizontal : p.start.z;
            };
            sample(left,z(left)); sample(right,z(right));
        }
        prefix+=p.length;
    }
    constant(std::min(f.departure+peer.duration,f.departure+prefix/f.speed),end,f.goal.z);
    // Signed altitude difference is linear on each flight leg. Check both
    // sides of every boundary, also covering discontinuous external inputs.
    // The extra (Va+Vb)*dt/4 for dt=2 certifies that the ORIGINAL sampler
    // would accept every interval at its first level; retain its decisions.
    double margin=separation+1e-4+(own.speed+f.speed)*.5+1e-7*scale;
    return low>margin || high<-margin;
}
template <class A, class B> struct PairCheck {
    const A &a;
    const B &b;
    double threshold, speed_bound, tolerance;
    double distance(double time) const {
        auto p = a.at(time), q = b.at(time);
        return std::hypot(std::hypot(p.x - q.x, p.y - q.y), p.z - q.z);
    }
    Separation interval(double left, double right, double dl, double dr) const {
        double middle = (left + right) / 2, dm = distance(middle);
        double closest = dl, at = left;
        if (dm < closest) {
            closest = dm;
            at = middle;
        }
        if (dr < closest) {
            closest = dr;
            at = right;
        }
        if (closest <= threshold)
            return {0, 0, at, closest};
        // Every time is within dt/4 of one sample; d is (Va+Vb)-Lipschitz.
        if (closest - speed_bound * (right - left) / 4 > threshold)
            return {};
        if (right - left <= tolerance || middle == left || middle == right)
            return {0, 1, at, closest};
        auto first = interval(left, middle, dl, dm);
        return first.safe ? interval(middle, right, dm, dr) : first;
    }
};
} // namespace

template <class A, class B>
Separation certify(const A &a, const B &b, double separation, double speed_bound,
                   double begin, double end, double step, double tolerance) {
    PairCheck<A, B> check{a, b, separation + 1e-4, speed_bound, tolerance};
    double left = begin, dl = check.distance(begin);
    if (dl <= check.threshold)
        return {0, 0, left, dl};
    while (left < end) {
        double right = std::min(end, left + step);
        if (right <= left)
            return {0, 1, left, dl};
        double dr = check.distance(right);
        auto result = check.interval(left, right, dl, dr);
        if (!result.safe)
            return result;
        left = right;
        dl = dr;
    }
    return {};
}

Separation check_separation(const TimedFlight &fa, const TimedFlight &fb, double separation,
                            double begin, double end, double step, double tolerance) {
    if (!std::isfinite(separation) || separation <= 0 || !std::isfinite(begin) || begin < 0 ||
        !std::isfinite(end) || end < begin || !std::isfinite(step) || !std::isfinite(tolerance) ||
        tolerance <= 0 || step < tolerance)
        throw std::invalid_argument("invalid separation-check limits");
    Motion a(fa), b(fb);
    return certify(a, b, separation, fa.speed + fb.speed, begin, end, step, tolerance);
}

Separation check_path_separation(const Path &path, double speed, double departure,
                                 const TimedFlight &peer, double separation,
                                 double step, double tolerance) {
    if (!std::isfinite(speed) || speed <= 0 || !std::isfinite(departure) || departure < 0 ||
        !std::isfinite(path.length) || path.length < 0 || !std::isfinite(separation) ||
        separation <= 0 || !std::isfinite(step) || !std::isfinite(tolerance) ||
        tolerance <= 0 || step < tolerance)
        throw std::invalid_argument("invalid timed edge");
    PathMotion own{path, speed, departure};
    Motion other(peer);
    return certify(own, other, separation, speed + peer.speed, departure,
                   departure + path.length / speed, step, tolerance);
}

struct TrafficChecker::Impl {
    const Traffic &traffic;
    std::vector<Motion> peers;
    std::vector<double> arrivals;
    explicit Impl(const Traffic &t) : traffic(t) {
        if (t.count<0 || (t.count && !t.flights) || !std::isfinite(t.speed) || t.speed<=0 ||
            !std::isfinite(t.departure) || t.departure<0 || !std::isfinite(t.separation) ||
            t.separation<=0 || !std::isfinite(t.horizon) || t.horizon<=t.departure)
            throw std::invalid_argument("invalid prepared traffic");
        peers.reserve(t.count);
        arrivals.reserve(t.count);
        for (int i=0; i<t.count; ++i) {
            peers.emplace_back(t.flights[i]);
            double arrival=t.flights[i].departure;
            for (int j=0; j<t.flights[i].count; ++j)
                arrival+=t.flights[i].paths[j].length/t.flights[i].speed;
            arrivals.push_back(arrival);
        }
    }
};
TrafficChecker::TrafficChecker(const Traffic &t) : data(std::make_unique<Impl>(t)) {}
TrafficChecker::~TrafficChecker() = default;
bool TrafficChecker::edge_free(const Path &p, double cost) const {
    const auto &t=data->traffic;
    double departure=t.departure+cost/t.speed, end=departure+p.length/t.speed;
    if (end>t.horizon) return false;
    PathMotion own(p,t.speed,departure);
    for (const auto &peer : data->peers)
        if (!vertically_certified(own,peer,t.separation) &&
            !certify(own,peer,t.separation,t.speed+peer.flight.speed,departure,end,2,.01).safe)
            return false;
    return true;
}
bool TrafficChecker::terminal_free(const Path &p, double cost) const {
    const auto &t=data->traffic;
    double departure=t.departure+cost/t.speed, arrival=departure+p.length/t.speed;
    TimedFlight tail{&p,1,t.speed,departure,t.start,t.goal};
    Motion own(tail);
    for (size_t i=0; i<data->peers.size(); ++i) {
        double end=std::min(t.horizon,data->arrivals[i]);
        if (end>arrival && !certify(own,data->peers[i],t.separation,
                                   t.speed+data->peers[i].flight.speed,arrival,end,2,.01).safe)
            return false;
    }
    return true;
}
} // namespace dubins
