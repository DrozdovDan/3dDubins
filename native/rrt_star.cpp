#include "dubins.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace dubins {
namespace {
struct GoalChoice {
    Path path;
    int static_status = 0; // 0=unchecked, 1=free, -1=blocked
};
struct GoalCache {
    std::vector<GoalChoice> choices;
    std::vector<int> lookup;
    bool initialized = false;
    std::vector<std::optional<Path>> partial;
    std::vector<unsigned char> computed;
};
struct Node {
    Pose pose;
    double cost = 0;
    int parent = -1;
    Path edge{};
    std::optional<Path> goal;
    std::vector<int> children;
    // Geometry/static checks do not depend on arrival time or tree parent.
    // Goal-biased samples reuse these SAME connections during parent search.
    std::shared_ptr<GoalCache> goal_cache;
    double goal_lower = -1;
    bool goal_deferred = false;
};
std::optional<Path> connect(const Pose &a, const Pose &b, const Options &o) {
    try {
        return plan(a, b, o.radius, o.max_angle);
    } catch (const std::invalid_argument &) {
        return {};
    }
}
double lower_bound(double distance, double dz, double radius, double sine) {
    // Every feasible edge satisfies L >= |dz| / sin(gamma_max), as well
    // as the triangle inequality. In steep 1 km missions this is far
    // tighter than Euclidean distance; never solve an Owen connection
    // when this conservative bound already proves it cannot compete.
    distance = std::max(distance, std::abs(dz) / sine);
    // Include the steering endpoint tolerance (fitted radius <= 2*R_min),
    // not just machine roundoff, before pruning any candidate.
    return std::max(0.0,
                    distance - 1e-7 * std::max(1.0, 2 * radius) - 1e-8 * std::max(1.0, distance));
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
                const std::vector<Pose> &samples, const Options &o, FirstSolution callback,
                const Traffic *traffic, double relative_gap) {
    if (roots.empty() || goals.empty() || samples.empty() || !std::isfinite(o.radius) ||
        !std::isfinite(o.max_angle) || !std::isfinite(o.neighbour_radius) ||
        !std::isfinite(o.collision_step) || !std::isfinite(o.time_limit) || o.radius <= 0 ||
        o.max_angle <= 0 || o.max_angle >= pi / 2 || o.neighbour_radius <= 0 ||
        o.collision_step <= 0 || world.rows <= 0 || world.columns <= 0 || !world.lower ||
        !world.upper || world.resolution <= 0 || !std::isfinite(world.resolution) ||
        !std::isfinite(world.x0) || !std::isfinite(world.y0) || !std::isfinite(world.margin) ||
        world.margin < 0 || world.boxes_count < 0 || (world.boxes_count && !world.boxes) ||
        !std::isfinite(relative_gap) || relative_gap < 0 || relative_gap > 1)
        throw std::invalid_argument("invalid RRT* input");
    if (traffic &&
        (roots.size() != 1 || traffic->count < 0 || (traffic->count && !traffic->flights) ||
         !std::isfinite(traffic->speed) || traffic->speed <= 0 ||
         !std::isfinite(traffic->departure) || traffic->departure < 0 ||
         !std::isfinite(traffic->separation) || traffic->separation <= 0 ||
         !std::isfinite(traffic->horizon) || traffic->horizon <= traffic->departure))
        throw std::invalid_argument("invalid traffic constraints or fixed root");
    if (traffic)
        for (const auto &c : {traffic->start, traffic->goal})
            if (!std::isfinite(c.x) || !std::isfinite(c.y) || !std::isfinite(c.z) ||
                !std::isfinite(c.radius) || c.radius <= 0)
                throw std::invalid_argument("invalid traffic circles");
    auto begin = std::chrono::steady_clock::now();
    const double sine = std::sin(o.max_angle);
    std::vector<Node> nodes;
    nodes.reserve(roots.size() + samples.size());
    for (const auto &p : roots) {
        Node node{};
        node.pose = p;
        nodes.push_back(std::move(node));
    }
    Result result;
    std::unique_ptr<TrafficChecker> prepared_traffic;
    if (traffic)
        prepared_traffic = std::make_unique<TrafficChecker>(*traffic);
    auto disk_distance = [](const Circle &a, const Circle &b) {
        return std::hypot(std::max(0.0, std::hypot(a.x - b.x, a.y - b.y) - a.radius - b.radius),
                          a.z - b.z);
    };
    if (traffic) {
        Path zero = plan(roots.front(), roots.front(), o.radius, o.max_angle);
        TimedFlight hold{&zero,         1, traffic->speed, traffic->departure, traffic->start,
                         traffic->start};
        for (int i = 0; i < traffic->count; ++i)
            if (disk_distance(traffic->goal, traffic->flights[i].goal) <=
                    traffic->separation + 1e-4 ||
                !check_separation(hold, traffic->flights[i], traffic->separation, 0,
                                  traffic->departure, 2, .01)
                     .safe)
                return result;
    }
    auto timed_free = [&](const Path &p, double cost) {
        return !prepared_traffic || prepared_traffic->edge_free(p, cost);
    };
    auto free = [&](const Path &p, double cost) {
        return world.path_free(p, o.collision_step, std::min(0.05, o.collision_step)) &&
               timed_free(p, cost);
    };
    auto goal_free = [&](const Path &p, double cost, bool known_static_free = false) {
        if ((!known_static_free &&
             !world.path_free(p, o.collision_step, std::min(0.05, o.collision_step))) ||
            !timed_free(p, cost))
            return false;
        return !prepared_traffic || prepared_traffic->terminal_free(p, cost);
    };
    std::vector<Path> choices;
    std::vector<int> goal_order;
    std::vector<int> goal_ids;
    std::vector<std::shared_ptr<GoalCache>> terminal_cache(goals.size());
    choices.reserve(goals.size());
    goal_order.reserve(goals.size());
    goal_ids.reserve(goals.size());
    auto goal_index = [&](const Pose &p) {
        for (int i = 0; i < static_cast<int>(goals.size()); ++i) {
            const auto &g = goals[i];
            if (p.x == g.x && p.y == g.y && p.z == g.z && p.yaw == g.yaw)
                return i;
        }
        return -1;
    };
    auto ensure_goal_cache = [&](Node &node) -> GoalCache & {
        if (!node.goal_cache) {
            int goal = goal_index(node.pose);
            if (goal >= 0)
                node.goal_cache = terminal_cache[goal];
            if (!node.goal_cache) {
                node.goal_cache = std::make_shared<GoalCache>();
                if (goal >= 0)
                    terminal_cache[goal] = node.goal_cache;
            }
        }
        return *node.goal_cache;
    };
    auto connect_goal = [&](Node &node) {
        node.goal.reset();
        if (node.goal_lower < 0) {
            node.goal_lower = std::numeric_limits<double>::infinity();
            for (const auto &g : goals)
                node.goal_lower = std::min(node.goal_lower,
                    lower_bound(std::hypot(node.pose.x-g.x, node.pose.y-g.y, node.pose.z-g.z),
                                node.pose.z-g.z, o.radius, sine));
        }
        if (!result.history.empty() && node.cost + node.goal_lower > result.history.back().cost) {
            node.goal_deferred = true;
            return;
        }
        node.goal_deferred = false;
        auto &cache = ensure_goal_cache(node);
        auto cached_goal = [&]() {
            for (auto &choice : cache.choices) {
                if (!choice.static_status)
                    choice.static_status = world.path_free(choice.path, o.collision_step,
                                                           std::min(0.05, o.collision_step))
                                               ? 1
                                               : -1;
                if (choice.static_status > 0 && goal_free(choice.path, node.cost, true)) {
                    node.goal = choice.path;
                    return;
                }
            }
        };
        if (cache.initialized) {
            cached_goal();
            return;
        }
        choices.clear();
        goal_order.clear();
        goal_ids.clear();
        for (int goal = 0; goal < static_cast<int>(goals.size()); ++goal) {
            auto edge = !cache.computed.empty() && cache.computed[goal]
                            ? cache.partial[goal] : connect(node.pose, goals[goal], o);
            if (edge) {
                goal_order.push_back(static_cast<int>(choices.size()));
                goal_ids.push_back(goal);
                choices.push_back(*edge);
            }
        }
        // Sort small indices, not whole Path objects. Explicit index ties
        // preserve the original stable goal ordering; buffers are reused.
        std::sort(goal_order.begin(), goal_order.end(), [&](int a, int b) {
            return choices[a].length < choices[b].length ||
                   (choices[a].length == choices[b].length && a < b);
        });
        cache.choices.reserve(goal_order.size());
        cache.lookup.assign(goals.size(), -1);
        for (int i : goal_order) {
            cache.lookup[goal_ids[i]] = static_cast<int>(cache.choices.size());
            cache.choices.push_back({choices[i], 0});
        }
        cache.initialized = true;
        cache.partial.clear();
        cache.computed.clear();
        cached_goal();
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
    std::vector<Path> incumbent;
    auto record = [&](int iteration) {
        int i = best();
        if (i < 0)
            return;
        double cost = nodes[i].cost + nodes[i].goal->length;
        if (result.history.empty() || cost < result.history.back().cost - 1e-8) {
            if (result.history.empty() && callback && callback(iteration))
                throw std::runtime_error("first-solution callback interrupted planning");
            result.history.push_back({iteration, cost});
            if (traffic) {
                // Retiming a subtree can invalidate its old goal connection.
                // Keep the best complete VALID path independently of the tree.
                incumbent = {*nodes[i].goal};
                for (int j = i; nodes[j].parent >= 0; j = nodes[j].parent)
                    incumbent.push_back(nodes[j].edge);
                std::reverse(incumbent.begin(), incumbent.end());
            }
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
    double route_bound = std::numeric_limits<double>::infinity();
    if (relative_gap > 0)
        for (const auto &a : roots)
            for (const auto &b : goals)
                route_bound = std::min(route_bound,
                    lower_bound(std::hypot(a.x-b.x, a.y-b.y, a.z-b.z),
                                a.z-b.z, o.radius, sine));
    for (int iteration = 1; iteration <= static_cast<int>(samples.size()); ++iteration) {
        // A COMPLETE, continuously checked incumbent is within the requested
        // gap of a physical lower bound. With no solution (or a loose bound)
        // keep searching up to the original cap. No wall-clock/random cutoff.
        if (relative_gap > 0 && !result.history.empty() &&
            result.history.back().cost <= (1 + relative_gap) * route_bound)
            break;
        if (o.time_limit >= 0 &&
            std::chrono::duration<double>(std::chrono::steady_clock::now() - begin).count() >=
                o.time_limit)
            break;
        result.iterations = iteration;
        const auto &sampled = samples[iteration - 1];
        if (!world.free(sampled))
            continue;
        int sampled_goal = goal_index(sampled);
        auto incoming = [&](int i) -> std::optional<Path> {
            if (sampled_goal >= 0) {
                auto &cache = ensure_goal_cache(nodes[i]);
                if (cache.initialized) {
                    int choice = cache.lookup[sampled_goal];
                    return choice < 0 ? std::optional<Path>{} : cache.choices[choice].path;
                }
                // Deferred terminals still receive repeated goal-biased samples.
                // Memoise only requested geometry, including unsupported edges.
                if (cache.partial.empty()) {
                    cache.partial.resize(goals.size());
                    cache.computed.assign(goals.size(), 0);
                }
                if (!cache.computed[sampled_goal]) {
                    cache.partial[sampled_goal] = connect(nodes[i].pose, sampled, o);
                    cache.computed[sampled_goal] = 1;
                }
                return cache.partial[sampled_goal];
            }
            return connect(nodes[i].pose, sampled, o);
        };
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
            bounds[i] = lower_bound(distance, nodes[i].pose.z - sampled.z, o.radius, sine);
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
            auto edge = incoming(i);
            if (!edge || edge->length <= 1e-8)
                continue;
            update_nearest(i, *edge);
            double cost = nodes[i].cost + edge->length;
            if (edge->length <= neighbourhood &&
                (cost < parent_cost || (cost == parent_cost && i < parent)) &&
                free(*edge, nodes[i].cost)) {
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
            // Examine the most promising lower bounds FIRST, not old roots
            // then every node in insertion order. The exact nearest metric
            // and node-index ties are unchanged, but its upper bound tightens
            // much earlier, avoiding expensive steering to distant nodes.
            candidates.clear();
            for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
                if (tried[i] || bounds[i] > nearest_length)
                    continue;
                candidates.push_back({i, bounds[i]});
            }
            std::sort(candidates.begin(), candidates.end(),
                      [](const Candidate &a, const Candidate &b) {
                          return a.bound < b.bound || (a.bound == b.bound && a.index < b.index);
                      });
            for (const auto &candidate : candidates) {
                if (candidate.bound > nearest_length)
                    break;
                int i = candidate.index;
                auto edge = incoming(i);
                if (edge && edge->length > 1e-8)
                    update_nearest(i, *edge);
            }
            if (nearest >= 0 && nearest_length > neighbourhood &&
                free(nearest_edge, nodes[nearest].cost)) {
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
            // Incoming/outgoing lower bounds are symmetric. The SAME
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
                nodes[index].cost + outward->length < nodes[i].cost - 1e-8 &&
                free(*outward, nodes[index].cost)) {
                std::vector<int> affected{i};
                if (traffic) {
                    bool valid = true;
                    for (int a = index; a >= 0; a = nodes[a].parent)
                        if (a == i)
                            valid = false;
                    double difference = nodes[index].cost + outward->length - nodes[i].cost;
                    for (size_t k = 0; k < affected.size() && valid; ++k) {
                        int current = affected[k];
                        if (current != i &&
                            !timed_free(nodes[current].edge, nodes[current].cost -
                                                                 nodes[current].edge.length +
                                                                 difference))
                            valid = false;
                        affected.insert(affected.end(), nodes[current].children.begin(),
                                        nodes[current].children.end());
                    }
                    if (!valid)
                        continue; // Reject atomically: never leave stale descendant times.
                } else {
                    for (size_t k = 0; k < affected.size(); ++k)
                        affected.insert(affected.end(), nodes[affected[k]].children.begin(),
                                        nodes[affected[k]].children.end());
                }
                rewire(nodes, i, index, *outward);
                for (int current : affected)
                    if (traffic || nodes[current].goal_deferred)
                        connect_goal(nodes[current]);
                ++result.rewires;
            }
        }
        connect_goal(nodes[index]);
        record(iteration);
    }
    result.nodes = static_cast<int>(nodes.size());
    int chosen = best();
    if (traffic && !incumbent.empty()) {
        result.paths = std::move(incumbent);
        auto nonzero = std::remove_if(result.paths.begin(), result.paths.end(),
                                      [](const Path &p) { return p.length <= 1e-8; });
        if (nonzero == result.paths.begin())
            result.paths.resize(1);
        else
            result.paths.erase(nonzero, result.paths.end());
        return result;
    }
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
