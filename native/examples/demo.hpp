#pragma once
#include "dubins.hpp"
#include <optional>
#include <string>
#include <vector>

namespace demo {
struct Loiter {
    double x, y, z, radius;
};
// Own the grids; create the borrowed World view after moving a Scene.
struct Terrain {
    double x0 = -180, y0 = -400, resolution = 10, margin = 5;
    int rows = 80, columns = 118;
    std::vector<double> heights, lower, upper;
    std::vector<dubins::Box> boxes;
    dubins::World view() const;
};
struct Scene {
    std::string id, title, description;
    std::vector<dubins::Path> paths;
    std::vector<std::string> waypoints;
    std::optional<Terrain> terrain;
    std::vector<Loiter> loiters;
    std::optional<dubins::Path> reference;
    bool reference_free = false;
    std::vector<dubins::History> history;
    double planning_ms = 0;
    int nodes = 0, rewires = 0, iterations = 0;
};
std::vector<std::string> expand_scenarios(const std::string &name);
Scene make_scene(const std::string &name, int iterations, unsigned seed);
void save_visualization(const std::vector<Scene> &scenes, const std::string &directory);
} // namespace demo
