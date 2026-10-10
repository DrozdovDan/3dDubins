#include "demo.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <stdexcept>

namespace demo {
using namespace dubins;
namespace {
constexpr double angle = pi / 12;
std::vector<Pose> circle(const Loiter &c, int direction) {
    std::vector<Pose> states;
    for (int i = 0; i < 8; ++i) {
        double phase = tau * i / 8;
        double yaw = std::fmod(phase + direction * pi / 2 + tau, tau);
        states.push_back(
            {c.x + c.radius * std::cos(phase), c.y + c.radius * std::sin(phase), c.z, yaw});
    }
    return states;
}
Loiter safe_loiter(const Terrain &terrain, double x, double y, double radius) {
    auto world = terrain.view();
    int r0, c0, r1, c1;
    if (!world.cell(x - radius, y - radius, r0, c0) || !world.cell(x + radius, y + radius, r1, c1))
        throw std::runtime_error("loiter disk reaches unknown map space");
    double low = -std::numeric_limits<double>::infinity(), high = -low;
    for (int r = r0; r <= r1; ++r)
        for (int c = c0; c <= c1; ++c) {
            double x0 = terrain.x0 + c * terrain.resolution,
                   y0 = terrain.y0 + r * terrain.resolution;
            double dx = std::max({x0 - x, 0.0, x - x0 - terrain.resolution});
            double dy = std::max({y0 - y, 0.0, y - y0 - terrain.resolution});
            if (std::hypot(dx, dy) > radius)
                continue;
            int i = r * terrain.columns + c;
            low = std::max(low, terrain.lower[i]);
            high = std::min(high, terrain.upper[i]);
        }
    if (!(low < high))
        throw std::runtime_error("no safe band over the whole loiter disk");
    Loiter result{x, y, (low + high) / 2, radius};
    for (const auto &box : terrain.boxes) {
        double m = terrain.margin;
        double dx = std::max({box.x0 - m - x, 0.0, x - box.x1 - m});
        double dy = std::max({box.y0 - m - y, 0.0, y - box.y1 - m});
        if (std::hypot(dx, dy) <= radius && box.z0 - m <= result.z && result.z <= box.z1 + m)
            throw std::runtime_error("loiter disk intersects a forbidden volume");
    }
    return result;
}
Scene altitude(const std::string &name) {
    Scene scene;
    scene.id = name;
    double z = name == "low" ? 220 : name == "medium" ? 320 : 720;
    scene.title = name == "low"      ? "Оуэн: малый перепад высоты"
                  : name == "medium" ? "Оуэн: средний перепад высоты"
                                     : "Оуэн: большой перепад высоты";
    scene.description =
        name == "low" ? "Обычный CSC: угол подъёма подбирается без удлинения пути."
        : name == "medium"
            ? "Добавочная дуга удлиняет путь до необходимой длины; радиус остаётся минимальным."
            : "Полные витки спирали и подбор радиуса: подъём под предельным углом 15°.";
    scene.waypoints = {"Старт", "Финиш"};
    auto began = std::chrono::steady_clock::now();
    scene.paths.push_back(plan({0, 0, 120, 0}, {600, 250, z, pi / 2}, 40, angle));
    scene.planning_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
    int expected = name == "low" ? 0 : name == "medium" ? 1 : 2;
    if (scene.paths.front().altitude_case != expected)
        throw std::runtime_error("wrong altitude example case");
    return scene;
}
Scene delivery() {
    Scene scene;
    scene.id = "delivery";
    scene.title = "Доставка с возвратом в депо";
    scene.description = "Депо → высотная точка → зона доставки → депо. Порядок точек задан; это "
                        "геометрия перелёта, не модель посадки.";
    scene.waypoints = {"Депо", "Высотная точка", "Доставка", "Депо: возврат"};
    std::vector<Pose> poses{
        {0, 0, 120, 0}, {500, 350, 380, pi / 2}, {1000, 0, 120, 0}, {0, 0, 120, pi}};
    auto began = std::chrono::steady_clock::now();
    for (std::size_t i = 1; i < poses.size(); ++i)
        scene.paths.push_back(plan(poses[i - 1], poses[i], 50, angle));
    scene.planning_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
    return scene;
}
Scene obstacles(const std::string &name, int iterations, unsigned seed) {
    Scene scene;
    scene.id = name;
    scene.terrain.emplace();
    auto &terrain = *scene.terrain;
    double min_clearance = 40, max_clearance = 110;
    if (name == "building" || name == "buildings") {
        scene.title = name == "building" ? "Обход одного здания" : "Обход трёх зданий";
        scene.description = "RRT* сам выбирает обход. Оранжевые объёмы нельзя перелететь внутри "
                            "разрешённой полосы высоты.";
        if (name == "building")
            terrain.boxes = {{320, -130, 0, 480, 130, 300}};
        else
            terrain.boxes = {{220, -180, 0, 330, 90, 240},
                             {470, -60, 0, 580, 220, 240},
                             {650, -240, 0, 720, -80, 240}};
    } else if (name == "hill_box") {
        scene.title = "Холм и запрещённый объём";
        scene.description = "Путь должен обходить объём и оставаться внутри полосы над рельефом.";
        terrain.boxes = {{320, -130, 0, 480, 130, 300}};
    } else if (name == "ridge") {
        scene.title = "Обход горного хребта";
        scene.description =
            "Зданий нет: прямое соединение блокирует сам рельеф. Евклидовы отступы — 30–100 м.";
        terrain.y0 = -500;
        terrain.rows = 100;
        min_clearance = 30;
        max_clearance = 100;
    } else {
        scene.title = "Подъём на возвышенность";
        scene.description = "Цель выше старта. Прямой Dubins-путь допустим, но высоту нужно "
                            "проверять относительно рельефа.";
    }
    for (int row = 0; row < terrain.rows; ++row)
        for (int col = 0; col < terrain.columns; ++col) {
            double x = terrain.x0 + (col + .5) * terrain.resolution,
                   y = terrain.y0 + (row + .5) * terrain.resolution, h = 0;
            if (name == "hill_box")
                h = 65 * std::exp(-std::pow((x - 400) / 140, 2) - std::pow(y / 190, 2));
            else if (name == "ridge")
                h = 170 * std::exp(-std::pow((x - 400) / 65, 2) - std::pow(y / 230, 2));
            else if (name == "climb")
                h = 120 / (1 + std::exp(-(x - 450) / 110));
            terrain.heights.push_back(h);
        }
    terrain.lower.resize(terrain.heights.size());
    terrain.upper.resize(terrain.heights.size());
    auto offset = [&](double distance, int upper, std::vector<double> &out) {
        if (dubins_offset(terrain.heights.data(), terrain.rows, terrain.columns, terrain.resolution,
                          distance, upper, out.data()))
            throw std::runtime_error(dubins_error());
    };
    offset(min_clearance, 1, terrain.lower);
    offset(max_clearance, 0, terrain.upper);
    scene.loiters = {safe_loiter(terrain, 0, 0, 40), safe_loiter(terrain, 800, 0, 40)};
    auto roots = circle(scene.loiters.front(), 1), goals = circle(scene.loiters.back(), -1),
         ccw = circle(scene.loiters.back(), 1);
    goals.insert(goals.end(), ccw.begin(), ccw.end());
    auto world = terrain.view();
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> x(terrain.x0,
                                             terrain.x0 + terrain.columns * terrain.resolution),
        y(terrain.y0, terrain.y0 + terrain.rows * terrain.resolution), yaw(-pi, pi), chance(0, 1);
    std::uniform_int_distribution<int> goal_index(0, static_cast<int>(goals.size()) - 1);
    auto began = std::chrono::steady_clock::now();
    std::vector<Pose> samples;
    samples.reserve(iterations);
    for (int i = 0; i < iterations; ++i) {
        if (chance(rng) < .15) {
            samples.push_back(goals[goal_index(rng)]);
            continue;
        }
        double sx = x(rng), sy = y(rng);
        int row, col;
        if (!world.cell(sx, sy, row, col) || !(terrain.lower[row * terrain.columns + col] <
                                               terrain.upper[row * terrain.columns + col])) {
            double nan = std::numeric_limits<double>::quiet_NaN();
            samples.push_back({nan, nan, nan, nan});
            continue;
        }
        int cell = row * terrain.columns + col;
        std::uniform_real_distribution<double> z(terrain.lower[cell], terrain.upper[cell]);
        samples.push_back({sx, sy, z(rng), yaw(rng)});
    }
    auto result = rrt_star(world, roots, goals, samples, {40, angle, 400, 10, -1});
    scene.planning_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
    if (result.paths.empty())
        throw std::runtime_error(name +
                                 ": no path within budget; increase --iterations or change --seed");
    scene.paths = std::move(result.paths);
    scene.history = std::move(result.history);
    scene.nodes = result.nodes;
    scene.rewires = result.rewires;
    scene.iterations = result.iterations;
    scene.waypoints = {"Стартовая окружность"};
    for (std::size_t i = 1; i < scene.paths.size(); ++i)
        scene.waypoints.push_back("Стык " + std::to_string(i));
    scene.waypoints.push_back("Целевая окружность");
    for (const auto &p : scene.paths)
        if (!world.path_free(p, 10, .05))
            throw std::runtime_error("final continuous collision check failed");
    try {
        scene.reference = plan(scene.paths.front().start, scene.paths.back().goal, 40, angle);
        scene.reference_free = world.path_free(*scene.reference, 10, .05);
    } catch (const std::invalid_argument &) { /* Unsupported reference: never fabricate one. */
    }
    return scene;
}
} // namespace
World Terrain::view() const {
    return {x0,
            y0,
            resolution,
            margin,
            rows,
            columns,
            static_cast<int>(boxes.size()),
            lower.data(),
            upper.data(),
            boxes.data()};
}
std::vector<std::string> expand_scenarios(const std::string &name) {
    if (name == "basic")
        return {"high", "building"};
    if (name == "altitude")
        return {"low", "medium", "high"};
    if (name == "obstacles")
        return {"building", "buildings", "hill_box", "ridge", "climb"};
    if (name == "all")
        return {"low",       "medium",   "high",  "delivery", "building",
                "buildings", "hill_box", "ridge", "climb"};
    for (const auto &id : {"low", "medium", "high", "delivery", "building", "buildings", "hill_box",
                           "ridge", "climb"})
        if (name == id)
            return {name};
    throw std::invalid_argument("unknown scenario: " + name);
}
Scene make_scene(const std::string &name, int iterations, unsigned seed) {
    if (name == "low" || name == "medium" || name == "high")
        return altitude(name);
    if (name == "delivery")
        return delivery();
    return obstacles(name, iterations, seed);
}
} // namespace demo
