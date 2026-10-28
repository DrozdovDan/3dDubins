#pragma once
#include "dubins.hpp"
#include <algorithm>
#include <cmath>

namespace dubins::detail {
// Prepared segment prefixes. XY arithmetic and distance subtraction match
// Path::pose_at; distance checks do not need the final heading wrapped.
struct Curve {
    const Path &path;
    struct Frame { double x, y, yaw, sine, cosine; };
    std::array<Frame, 4> frames{};
    explicit Curve(const Path &p) : path(p) {
        double x=p.start.x, y=p.start.y, yaw=p.start.yaw;
        for (int i=0; i<p.count; ++i) {
            auto &f=frames[i];
            f={x,y,yaw,std::sin(yaw),std::cos(yaw)};
            const auto &s=p.segments[i];
            if (s.kind==2) {
                x+=s.length*f.cosine;
                y+=s.length*f.sine;
            } else {
                double sign=s.kind==0 ? 1 : -1;
                double next=yaw+sign*s.length/p.radius;
                x+=sign*p.radius*(std::sin(next)-f.sine);
                y+=sign*p.radius*(f.cosine-std::cos(next));
                yaw=next;
            }
        }
    }
    Pose at(double distance) const {
        if (path.horizontal==0) return path.start;
        double remaining=distance;
        Pose p=path.start;
        for (int i=0; i<path.count; ++i) {
            const auto &s=path.segments[i];
            if (remaining>s.length && i+1<path.count) {
                remaining-=s.length;
                continue;
            }
            const auto &f=frames[i];
            double d=std::min(remaining,s.length);
            p.x=f.x; p.y=f.y; p.yaw=f.yaw;
            if (s.kind==2) {
                p.x+=d*f.cosine;
                p.y+=d*f.sine;
            } else {
                double sign=s.kind==0 ? 1 : -1;
                p.yaw+=sign*d/path.radius;
                p.x+=sign*path.radius*(std::sin(p.yaw)-f.sine);
                p.y+=sign*path.radius*(f.cosine-std::cos(p.yaw));
            }
            break;
        }
        p.z=path.start.z+(path.goal.z-path.start.z)*distance/path.horizontal;
        return p;
    }
};
}
