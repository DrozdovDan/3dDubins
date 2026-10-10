#include "dubins.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dubins {
namespace {
struct Node {
    Pose pose;
    double cost = 0;
    int parent = -1;
    Path edge{};
    std::optional<Path> goal;
    std::vector<int> children;
};
std::optional<Path> connect(const Pose &a, const Pose &b, const Options &o) {
    try {
        return plan(a, b, o.radius, o.max_angle);
    } catch (const std::invalid_argument &) {
        return {};
    }
}
double lower_bound(double distance, double radius) {
    // Triangle inequality gives a conservative lower bound on ANY 3D path.
    // Include the steering endpoint tolerance (fitted radius <= 2*R_min),
    // not just machine roundoff, before pruning any candidate.
    return std::max(0.0,
                    distance - 1e-7 * std::max(1.0, 2 * radius) - 1e-10 * std::max(1.0, distance));
}
void rewire(std::vector<Node> &nodes, int index, int parent, const Path &edge) {
    auto &node = nodes[index];
    for (int a = parent; a >= 0; a = nodes[a].parent)
        if (a == index)
            throw std::runtime_error("rewiring would create a cycle");
    auto &old = nodes[node.parent].children;
    old.erase(std::find(old.begin(), old.end(), index));
    nodes[parent].children.push_back(index);
    double difference = nodes[parent].cost + edge.length - node.cost;
    node.parent = parent;
    node.edge = edge;
    std::vector<int> stack{index};
    while (!stack.empty()) {
        int current = stack.back();
        stack.pop_back();
        nodes[current].cost += difference;
        stack.insert(stack.end(), nodes[current].children.begin(), nodes[current].children.end());
    }
}
} // namespace
Result rrt_star(const World &world, const std::vector<Pose> &roots, const std::vector<Pose> &goals,
                const std::vector<Pose> &samples, const Options &o, FirstSolution callback) {
    if (roots.empty() || goals.empty() || samples.empty() || !std::isfinite(o.radius) ||
        !std::isfinite(o.max_angle) || !std::isfinite(o.neighbour_radius) ||
        !std::isfinite(o.collision_step) || !std::isfinite(o.time_limit) || o.radius <= 0 ||
        o.max_angle <= 0 || o.max_angle >= pi / 2 || o.neighbour_radius <= 0 ||
        o.collision_step <= 0 || world.rows <= 0 || world.columns <= 0 || !world.lower ||
        !world.upper || world.resolution <= 0 || !std::isfinite(world.resolution) ||
        !std::isfinite(world.x0) || !std::isfinite(world.y0) || !std::isfinite(world.margin) ||
        world.margin < 0 || world.boxes_count < 0 || (world.boxes_count && !world.boxes))
        throw std::invalid_argument("invalid RRT* input");
    auto begin = std::chrono::steady_clock::now();
    std::vector<Node> nodes;
    nodes.reserve(roots.size() + samples.size());
    for (const auto &p : roots) {
        Node node{};
        node.pose = p;
        nodes.push_back(std::move(node));
    }
    Result result;
    auto free = [&](const Path &p) {
        return world.path_free(p, o.collision_step, std::min(0.05, o.collision_step));
    };
    std::vector<Path> choices;
    std::vector<int> goal_order;
    choices.reserve(goals.size());
    goal_order.reserve(goals.size());
    auto connect_goal = [&](Node &node) {
        choices.clear();
        goal_order.clear();
        for (const auto &goal : goals)
            if (auto edge = connect(node.pose, goal, o)) {
                goal_order.push_back(static_cast<int>(choices.size()));
                choices.push_back(*edge);
            }
        // Sort small indices, not whole Path objects. Explicit index ties
        // preserve the original stable goal ordering; buffers are reused.
        std::sort(goal_order.begin(), goal_order.end(), [&](int a, int b) {
            return choices[a].length < choices[b].length ||
                   (choices[a].length == choices[b].length && a < b);
        });
        for (int i : goal_order)
            if (free(choices[i])) {
                node.goal = choices[i];
                return;
            }
    };
    auto best = [&]() {
        int index = -1;
        double cost = std::numeric_limits<double>::infinity();
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
            if (nodes[i].goal) {
                double candidate = nodes[i].cost + nodes[i].goal->length;
                if (candidate < cost) {
                    cost = candidate;
                    index = i;
                }
            }
        return index;
    };
    auto record = [&](int iteration) {
        int i = best();
        if (i < 0)
            return;
        double cost = nodes[i].cost + nodes[i].goal->length;
        if (result.history.empty() || cost < result.history.back().cost - 1e-8) {
            if (result.history.empty() && callback && callback(iteration))
                throw std::runtime_error("first-solution callback interrupted planning");
            result.history.push_back({iteration, cost});
        }
    };
    for (auto &node : nodes)
        connect_goal(node);
    record(0);
    double scale = 2 * std::hypot((world.x0 + world.columns * world.resolution) - world.x0,
                                  (world.y0 + world.rows * world.resolution) - world.y0);
    struct Candidate {
        int index;
        double bound;
    };
    std::vector<Candidate> candidates;
    candidates.reserve(roots.size() + samples.size());
    std::vector<double> bounds;
    bounds.reserve(roots.size() + samples.size());
    std::vector<unsigned char> tried;
    tried.reserve(roots.size() + samples.size());
    for (int iteration = 1; iteration <= static_cast<int>(samples.size()); ++iteration) {
        if (o.time_limit >= 0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() >=
                o.time_limit)
            break;
        result.iterations = iteration;
        const auto &sampled = samples[iteration - 1];
        if (!world.free(sampled))
            continue;
        int n = static_cast<int>(nodes.size()) + 1;
        double neighbourhood =
            std::min(o.neighbour_radius, scale * std::pow(std::log(n) / n, 0.25));
        candidates.clear();
        bounds.resize(nodes.size());
        tried.assign(nodes.size(), 0);
        int nearest = -1;
        double nearest_length = std::numeric_limits<double>::infinity();
        Path nearest_edge{};
        for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
            double distance = std::hypot(nodes[i].pose.x - sampled.x, nodes[i].pose.y - sampled.y,
                                         nodes[i].pose.z - sampled.z);
            bounds[i] = lower_bound(distance, o.radius);
            if (bounds[i] <= neighbourhood)
                candidates.push_back({i, nodes[i].cost + bounds[i]});
        }
        std::sort(candidates.begin(), candidates.end(), [](const Candidate &a, const Candidate &b) {
            return a.bound < b.bound || (a.bound == b.bound && a.index < b.index);
        });
        auto update_nearest = [&](int i, const Path &edge) {
            if (edge.length < nearest_length || (edge.length == nearest_length && i < nearest)) {
                nearest = i;
                nearest_length = edge.length;
                nearest_edge = edge;
            }
        };
        int parent = -1;
        Path parent_edge{};
        double parent_cost = std::numeric_limits<double>::infinity();
        // Lazy, EXACT parent selection: once a feasible parent is found,
        // cost-to-come + the conservative distance bound proves which of
        // the remaining candidates cannot beat it. Equal-cost candidates
        // retain the original node-index tie order. No approximate edges.
        for (const auto &candidate : candidates) {
            if (candidate.bound > parent_cost)
                break;
            int i = candidate.index;
            tried[i] = 1;
            auto edge = connect(nodes[i].pose, sampled, o);
            if (!edge || edge->length <= 1e-8)
                continue;
            update_nearest(i, *edge);
            double cost = nodes[i].cost + edge->length;
            if (edge->length <= neighbourhood &&
                (cost < parent_cost || (cost == parent_cost && i < parent)) && free(*edge)) {
                parent = i;
                parent_cost = cost;
                parent_edge = *edge;
            }
        }
        // A feasible neighbour implies the exact nearest is also inside
        // the neighbourhood, so it cannot add another parent candidate.
        // Otherwise preserve the original single-nearest fallback, NOT
        // a nearest-FREE fallback (which would change the tree).
        if (parent < 0) {
            for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
                if (tried[i] || bounds[i] > nearest_length)
                    continue;
                auto edge = connect(nodes[i].pose, sampled, o);
                if (edge && edge->length > 1e-8)
                    update_nearest(i, *edge);
            }
            if (nearest >= 0 && nearest_length > neighbourhood && free(nearest_edge)) {
                parent = nearest;
                parent_edge = nearest_edge;
                parent_cost = nodes[parent].cost + parent_edge.length;
            }
        }
        if (parent < 0)
            continue;
        int index = static_cast<int>(nodes.size());
        Node node{};
        node.pose = sampled;
        node.parent = parent;
        node.edge = parent_edge;
        node.cost = parent_cost;
        nodes.push_back(std::move(node));
        nodes[parent].children.push_back(index);
        for (int i = static_cast<int>(roots.size()); i < index; ++i) {
            // Incoming/outgoing Euclidean bounds are symmetric. The SAME
            // distances were already evaluated for parent selection.
            double lower = bounds[i];
            if (lower > neighbourhood)
                continue;
            if (nodes[index].cost + lower >= nodes[i].cost - 1e-8)
                continue;
            // Retain the old exact Euclidean neighbourhood boundary, not
            // the slightly relaxed numerical pruning bound.
            if (std::hypot(nodes[i].pose.x - sampled.x, nodes[i].pose.y - sampled.y,
                           nodes[i].pose.z - sampled.z) > neighbourhood)
                continue;
            auto outward = connect(sampled, nodes[i].pose, o);
            if (outward && outward->length <= neighbourhood &&
                nodes[index].cost + outward->length < nodes[i].cost - 1e-8 && free(*outward)) {
                rewire(nodes, i, index, *outward);
                ++result.rewires;
            }
        }
        connect_goal(nodes[index]);
        record(iteration);
    }
    result.nodes = static_cast<int>(nodes.size());
    int chosen = best();
    if (chosen < 0)
        return result; // C ABI translates this to NoPathError.
    result.paths.push_back(*nodes[chosen].goal);
    while (nodes[chosen].parent >= 0) {
        result.paths.push_back(nodes[chosen].edge);
        chosen = nodes[chosen].parent;
    }
    std::reverse(result.paths.begin(), result.paths.end());
    auto nonzero = std::remove_if(result.paths.begin(), result.paths.end(),
                                  [](const Path &p) { return p.length <= 1e-8; });
    if (nonzero == result.paths.begin())
        result.paths.resize(1);
    else
        result.paths.erase(nonzero, result.paths.end());
    return result;
}
} // namespace dubins
