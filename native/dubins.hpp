#pragma once

#include <array>
#include <cstdint>
#include <vector>

#if defined(_WIN32) && defined(DUBINS_BUILDING_LIBRARY)
#define DUBINS_API __declspec(dllexport)
#elif defined(_WIN32) && defined(DUBINS_USING_LIBRARY)
#define DUBINS_API __declspec(dllimport)
#elif defined(_WIN32)
#define DUBINS_API
#else
#define DUBINS_API __attribute__((visibility("default")))
#endif

namespace dubins {
constexpr double pi = 3.141592653589793238462643383279502884;
constexpr double tau = 2 * pi;
struct Pose {
    double x, y, z, yaw;
};
struct Segment {
    double length;
    int kind;
}; // 0=L, 1=R, 2=S
// Fixed-size paths avoid allocation in the steering/nearest-neighbour loops.
struct Path {
    Pose start, goal;
    double radius, minimum_radius, max_angle, gamma, extension, horizontal, length;
    std::array<Segment, 4> segments{};
    int count = 0, turns = 0, altitude_case = 0, base_word = 0;
    std::array<char, 5> word{};
    DUBINS_API Pose pose_at(double distance) const;
};
struct Box {
    double x0, y0, z0, x1, y1, z1;
};
// Borrowed row-major arrays; their owner must outlive a planning call.
struct World {
    double x0, y0, resolution, margin;
    int rows, columns, boxes_count;
    const double *lower, *upper;
    const Box *boxes;
    DUBINS_API bool cell(double x, double y, int &row, int &column) const;
    DUBINS_API bool free(const Pose &pose) const;
    DUBINS_API bool path_free(const Path &path, double step, double minimum_step) const;
};
struct Options {
    double radius, max_angle, neighbour_radius, collision_step, time_limit;
};
struct History {
    int iteration;
    double cost;
};
struct Result {
    std::vector<Path> paths;
    std::vector<History> history;
    int iterations = 0, nodes = 0, rewires = 0;
};
// Return nonzero to abort (e.g. if the Python callback raises an exception).
using FirstSolution = int (*)(int);
DUBINS_API Path plan(const Pose &start, const Pose &goal, double radius, double max_angle);
// The sample stream can come from any RNG. Python supplies its original
// seeded stream, so comparison does not conflate language and sampling.
DUBINS_API Result rrt_star(const World &, const std::vector<Pose> &roots,
                           const std::vector<Pose> &goals, const std::vector<Pose> &samples,
                           const Options &, FirstSolution callback = nullptr);
} // namespace dubins

extern "C" {
DUBINS_API int dubins_abi_version();
DUBINS_API const char *dubins_error();
DUBINS_API int dubins_plan(const dubins::Pose *start, const dubins::Pose *goal, double radius,
                           double max_angle, dubins::Path *out);
// Returns 1=free, 0=blocked, negative status code on invalid input/error.
DUBINS_API int dubins_check(const dubins::World *world, const dubins::Path *path, double step,
                            double minimum_step);
DUBINS_API int dubins_offset(const double *heights, int rows, int columns, double resolution,
                             double distance, int upper_bound, double *out);
// Caller provides iterations+1 Path and History slots. Inputs must remain
// alive until the call returns. Status: 0=OK, 1=invalid input, 2=runtime error.
DUBINS_API int dubins_rrt(const dubins::World *world, const dubins::Pose *roots, int roots_count,
                          const dubins::Pose *goals, int goals_count, const dubins::Pose *samples,
                          int iterations, const dubins::Options *options, dubins::Path *paths,
                          int *path_count, dubins::History *history, int *history_count,
                          int *completed, int *nodes, int *rewires, dubins::FirstSolution callback);
}
