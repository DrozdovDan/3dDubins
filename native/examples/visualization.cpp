#include "demo.hpp"
#include "viewer.hpp" // Offline page embedded by CMake, no runtime asset lookup.

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace demo {
namespace {
std::string quoted(const std::string &text) {
    std::ostringstream out;
    out << '"';
    for (unsigned char c : text) {
        if (c == '"' || c == '\\')
            out << '\\' << c;
        else if (c < 32 || c == '<' || c == '>' || c == '&')
            out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c)
                << std::dec;
        else
            out << c;
    }
    return out.str() + '"';
}

void numbers(std::ostream &out, const std::vector<double> &values) {
    out << '[';
    for (std::size_t i = 0; i < values.size(); ++i) {
        if (i)
            out << ',';
        out << values[i];
    }
    out << ']';
}

struct Point {
    dubins::Pose pose;
    double s;
    int leg;
};
std::vector<Point> sample(const std::vector<dubins::Path> &paths) {
    std::vector<Point> points;
    double cumulative = 0;
    for (std::size_t leg = 0; leg < paths.size(); ++leg) {
        const auto &path = paths[leg];
        int n = std::max(1, static_cast<int>(std::ceil(path.horizontal / 4)));
        std::vector<double> distances{0, path.horizontal};
        for (int i = 1; i < n; ++i)
            distances.push_back(path.horizontal * i / n);
        double join = 0;
        for (int i = 0; i < path.count; ++i) {
            join += path.segments[i].length;
            distances.push_back(std::min(join, path.horizontal));
        }
        std::sort(distances.begin(), distances.end());
        distances.erase(std::unique(distances.begin(), distances.end()), distances.end());
        for (double d : distances) {
            if (d == 0 && !points.empty())
                continue;
            double s = cumulative + (path.horizontal ? path.length * d / path.horizontal : 0);
            points.push_back({path.pose_at(d), s, static_cast<int>(leg)});
        }
        cumulative += path.length;
    }
    return points;
}

void points_json(std::ostream &out, const std::vector<Point> &points, const Terrain *terrain) {
    out << '[';
    for (std::size_t i = 0; i < points.size(); ++i) {
        if (i)
            out << ',';
        const auto &p = points[i];
        out << '[' << p.pose.x << ',' << p.pose.y << ',' << p.pose.z << ',' << p.pose.yaw << ','
            << p.s;
        if (terrain) {
            int r, c;
            if (!terrain->view().cell(p.pose.x, p.pose.y, r, c))
                throw std::runtime_error("visual sample outside terrain");
            int cell = r * terrain->columns + c;
            out << ',' << terrain->heights[cell] << ',' << terrain->lower[cell] << ','
                << terrain->upper[cell];
        } else
            out << ",null,null,null";
        out << ',' << p.leg << ']';
    }
    out << ']';
}

void csv(const std::filesystem::path &filename, const std::vector<Point> &points,
         const Terrain *terrain) {
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(filename);
    out << std::setprecision(17)
        << "east_m,north_m,altitude_m,yaw_rad,distance_3d_m,leg,ground_m,lower_m,upper_m\n";
    for (const auto &p : points) {
        out << p.pose.x << ',' << p.pose.y << ',' << p.pose.z << ',' << p.pose.yaw << ',' << p.s
            << ',' << p.leg + 1;
        if (terrain) {
            int r, c;
            terrain->view().cell(p.pose.x, p.pose.y, r, c);
            int i = r * terrain->columns + c;
            out << ',' << terrain->heights[i] << ',' << terrain->lower[i] << ','
                << terrain->upper[i];
        } else
            out << ",,,";
        out << '\n';
    }
}

std::string scene_json(const Scene &scene, const std::vector<Point> &points) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"id\":" << quoted(scene.id)
        << ",\"title\":" << quoted(scene.title) << ",\"description\":" << quoted(scene.description)
        << ",\"ms\":" << scene.planning_ms << ",\"nodes\":" << scene.nodes
        << ",\"rewires\":" << scene.rewires << ",\"iterations\":" << scene.iterations
        << ",\"points\":";
    points_json(out, points, scene.terrain ? &*scene.terrain : nullptr);
    out << ",\"legs\":[";
    for (std::size_t i = 0; i < scene.paths.size(); ++i) {
        if (i)
            out << ',';
        const auto &p = scene.paths[i];
        out << "{\"word\":" << quoted(p.word.data()) << ",\"case\":" << p.altitude_case
            << ",\"length\":" << p.length << ",\"radius\":" << p.radius
            << ",\"minimum\":" << p.minimum_radius << ",\"gamma\":" << p.gamma * 180 / dubins::pi
            << ",\"turns\":" << p.turns << '}';
    }
    out << "],\"waypoints\":[";
    double s = 0;
    for (std::size_t i = 0; i <= scene.paths.size(); ++i) {
        if (i) {
            out << ',';
            s += scene.paths[i - 1].length;
        }
        const auto &p = i == 0 ? scene.paths.front().start : scene.paths[i - 1].goal;
        out << '[' << quoted(scene.waypoints.at(i)) << ',' << p.x << ',' << p.y << ',' << p.z << ','
            << s << ']';
    }
    out << "],\"loiters\":[";
    for (std::size_t i = 0; i < scene.loiters.size(); ++i) {
        if (i)
            out << ',';
        const auto &c = scene.loiters[i];
        out << '[' << c.x << ',' << c.y << ',' << c.z << ',' << c.radius << ']';
    }
    out << "],\"history\":[";
    for (std::size_t i = 0; i < scene.history.size(); ++i) {
        if (i)
            out << ',';
        out << '[' << scene.history[i].iteration << ',' << scene.history[i].cost << ']';
    }
    out << "],\"referenceFree\":" << (scene.reference_free ? "true" : "false") << ",\"reference\":";
    if (scene.reference)
        points_json(out, sample({*scene.reference}), nullptr);
    else
        out << "[]";
    out << ",\"terrain\":";
    if (scene.terrain) {
        const auto &t = *scene.terrain;
        out << "{\"x0\":" << t.x0 << ",\"y0\":" << t.y0 << ",\"step\":" << t.resolution
            << ",\"rows\":" << t.rows << ",\"columns\":" << t.columns << ",\"margin\":" << t.margin
            << ",\"heights\":";
        numbers(out, t.heights);
        out << ",\"boxes\":[";
        for (std::size_t i = 0; i < t.boxes.size(); ++i) {
            if (i)
                out << ',';
            const auto &b = t.boxes[i];
            out << '[' << b.x0 << ',' << b.y0 << ',' << b.z0 << ',' << b.x1 << ',' << b.y1 << ','
                << b.z1 << ']';
        }
        out << "]}";
    } else
        out << "null";
    return out.str() + '}';
}
} // namespace

void save_visualization(const std::vector<Scene> &scenes, const std::string &directory) {
    if (scenes.empty())
        throw std::invalid_argument("no scenes to visualize");
    std::filesystem::create_directories(directory);
    std::string json = "[";
    for (std::size_t i = 0; i < scenes.size(); ++i) {
        if (i)
            json += ',';
        auto points = sample(scenes[i].paths);
        csv(std::filesystem::path(directory) / (scenes[i].id + ".csv"), points,
            scenes[i].terrain ? &*scenes[i].terrain : nullptr);
        json += scene_json(scenes[i], points);
    }
    json += ']';
    std::string html = viewer_template;
    const std::string marker = "__DUBINS_SCENES__";
    auto position = html.find(marker);
    if (position == std::string::npos)
        throw std::runtime_error("missing data placeholder in viewer");
    html.replace(position, marker.size(), json);
    std::ofstream out;
    out.exceptions(std::ios::failbit | std::ios::badbit);
    out.open(std::filesystem::path(directory) / "index.html", std::ios::binary);
    out << html;
}
} // namespace demo
