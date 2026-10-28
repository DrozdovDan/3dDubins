#include "dubins.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace {
thread_local std::string error;
template <class Function> int guarded(Function f) {
    error.clear();
    try {
        f();
        return 0;
    } catch (const std::invalid_argument &e) {
        error = e.what();
        return 1;
    } catch (const std::exception &e) {
        error = e.what();
        return 2;
    } catch (...) {
        error = "unknown native exception";
        return 2;
    }
}
} // namespace
extern "C" {
int dubins_abi_version() { return 1; }
const char *dubins_error() { return error.c_str(); }
int dubins_plan(const dubins::Pose *a, const dubins::Pose *b, double r, double g,
                dubins::Path *out) {
    return guarded([&] {
        if (!a || !b || !out)
            throw std::invalid_argument("null path argument");
        *out = dubins::plan(*a, *b, r, g);
    });
}
int dubins_check(const dubins::World *w, const dubins::Path *p, double step, double minimum) {
    int free = 0;
    int status = guarded([&] {
        if (!w || !p || !w->lower || !w->upper || w->rows <= 0 || w->columns <= 0 ||
            !std::isfinite(w->resolution) || w->resolution <= 0)
            throw std::invalid_argument("invalid collision grid");
        free = w->path_free(*p, step, minimum);
    });
    return status ? -status : free;
}
int dubins_offset(const double *height, int rows, int columns, double resolution, double distance,
                  int upper, double *out) {
    return guarded([&] {
        if (!height || !out || rows <= 0 || columns <= 0 || resolution <= 0 ||
            !std::isfinite(resolution) || distance < 0 || !std::isfinite(distance))
            throw std::invalid_argument("invalid offset grid");
        int reach = static_cast<int>(std::min(std::ceil(distance / resolution) + 1,
                                              static_cast<double>(std::max(rows, columns))));
        struct Kernel {
            int row, col;
            double z;
        };
        std::vector<Kernel> kernel;
        for (int dr = -reach; dr <= reach; ++dr)
            for (int dc = -reach; dc <= reach; ++dc) {
                double dx = upper ? std::max(0, std::abs(dc) - 1) : std::abs(dc);
                double dy = upper ? std::max(0, std::abs(dr) - 1) : std::abs(dr);
                double lateral = std::hypot(dx, dy) * resolution;
                if (lateral <= distance)
                    kernel.push_back(
                        {dr, dc,
                         std::sqrt(std::max(0.0, distance * distance - lateral * lateral))});
            }
        for (int r = 0; r < rows; ++r)
            for (int c = 0; c < columns; ++c) {
                double best = -INFINITY;
                for (const auto &k : kernel)
                    if (r + k.row >= 0 && r + k.row < rows && c + k.col >= 0 && c + k.col < columns)
                        best = std::max(best, height[(r + k.row) * columns + c + k.col] + k.z);
                out[r * columns + c] = best;
            }
    });
}
int dubins_rrt(const dubins::World *w, const dubins::Pose *roots, int roots_count,
               const dubins::Pose *goals, int goals_count, const dubins::Pose *samples,
               int iterations, const dubins::Options *options, dubins::Path *paths, int *path_count,
               dubins::History *history, int *history_count, int *completed, int *nodes,
               int *rewires, dubins::FirstSolution callback) {
    return guarded([&] {
        if (!w || !roots || !goals || !samples || !options || !paths || !path_count || !history ||
            !history_count || !completed || !nodes || !rewires || roots_count <= 0 ||
            goals_count <= 0 || iterations <= 0)
            throw std::invalid_argument("invalid RRT* buffers");
        auto result =
            dubins::rrt_star(*w, {roots, roots + roots_count}, {goals, goals + goals_count},
                             {samples, samples + iterations}, *options, callback);
        std::copy(result.paths.begin(), result.paths.end(), paths);
        std::copy(result.history.begin(), result.history.end(), history);
        *path_count = static_cast<int>(result.paths.size());
        *history_count = static_cast<int>(result.history.size());
        *completed = result.iterations;
        *nodes = result.nodes;
        *rewires = result.rewires;
    });
}
int dubins_fleet_check(const dubins::TimedFlight *a, const dubins::TimedFlight *b,
                      double separation, double begin, double end, double step,
                      double tolerance, dubins::Separation *out) {
    return guarded([&] {
        if (!a || !b || !out)
            throw std::invalid_argument("null fleet-check argument");
        *out = dubins::check_separation(*a, *b, separation, begin, end, step, tolerance);
    });
}
int dubins_rrt_traffic(const dubins::World *w, const dubins::Pose *roots, int roots_count,
                      const dubins::Pose *goals, int goals_count, const dubins::Pose *samples,
                      int iterations, const dubins::Options *options, dubins::Path *paths, int *path_count,
                      dubins::History *history, int *history_count, int *completed, int *nodes,
                      int *rewires, dubins::FirstSolution callback, const dubins::Traffic *traffic) {
    return guarded([&] {
        if (!w || !roots || !goals || !samples || !options || !paths || !path_count || !history ||
            !history_count || !completed || !nodes || !rewires || !traffic || roots_count != 1 ||
            goals_count <= 0 || iterations <= 0)
            throw std::invalid_argument("invalid traffic RRT* buffers");
        auto result = dubins::rrt_star(*w, {roots, roots + roots_count}, {goals, goals + goals_count},
                                       {samples, samples + iterations}, *options, callback, traffic);
        std::copy(result.paths.begin(), result.paths.end(), paths);
        std::copy(result.history.begin(), result.history.end(), history);
        *path_count = static_cast<int>(result.paths.size());
        *history_count = static_cast<int>(result.history.size());
        *completed = result.iterations;
        *nodes = result.nodes;
        *rewires = result.rewires;
    });
}
int dubins_rrt_adaptive(const dubins::World *w, const dubins::Pose *roots, int roots_count,
                       const dubins::Pose *goals, int goals_count, const dubins::Pose *samples,
                       int iterations, const dubins::Options *options, dubins::Path *paths, int *path_count,
                       dubins::History *history, int *history_count, int *completed, int *nodes,
                       int *rewires, dubins::FirstSolution callback, const dubins::Traffic *traffic,
                       double relative_gap) {
    return guarded([&] {
        if (!w || !roots || !goals || !samples || !options || !paths || !path_count || !history ||
            !history_count || !completed || !nodes || !rewires || roots_count <= 0 ||
            goals_count <= 0 || iterations <= 0)
            throw std::invalid_argument("invalid adaptive RRT* buffers");
        auto result = dubins::rrt_star(*w, {roots, roots + roots_count}, {goals, goals + goals_count},
                                     {samples, samples + iterations}, *options, callback, traffic,
                                     relative_gap);
        std::copy(result.paths.begin(), result.paths.end(), paths);
        std::copy(result.history.begin(), result.history.end(), history);
        *path_count = static_cast<int>(result.paths.size());
        *history_count = static_cast<int>(result.history.size());
        *completed = result.iterations;
        *nodes = result.nodes;
        *rewires = result.rewires;
    });
}
}
