// Four standalone executables share this CLI; no Python is used at runtime.
#include "examples/demo.hpp"
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

#ifndef DUBINS_DEFAULT_SCENARIO
#define DUBINS_DEFAULT_SCENARIO "basic"
#endif

int main(int argc, char **argv) {
    try {
        std::string scenario = DUBINS_DEFAULT_SCENARIO, output;
        int iterations = 600;
        unsigned seed = 7;
        for (int i = 1; i < argc; ++i) {
            std::string key = argv[i];
            if (key == "--help") {
                std::cout
                    << "Pure C++ Owen / RRT* examples with offline HTML visualization.\n"
                    << "  --scenario basic|altitude|delivery|obstacles|all\n"
                    << "             low|medium|high|building|buildings|hill_box|ridge|climb\n"
                    << "  --iterations N (default 600; RRT* only)\n"
                    << "  --seed N       (default 7; C++ RNG, not CPython RNG)\n"
                    << "  --output DIR   (default outputs/cpp/<scenario>)\n";
                return 0;
            }
            if (i + 1 >= argc)
                throw std::invalid_argument("missing value for " + key);
            std::string value = argv[++i];
            if (key == "--scenario")
                scenario = value;
            else if (key == "--output")
                output = value;
            else if (key == "--iterations" || key == "--seed") {
                std::size_t used = 0;
                long long n = std::stoll(value, &used);
                if (used != value.size() || n < (key == "--iterations" ? 1 : 0) || n > 1000000)
                    throw std::invalid_argument("invalid value for " + key);
                if (key == "--iterations")
                    iterations = static_cast<int>(n);
                else
                    seed = static_cast<unsigned>(n);
            } else
                throw std::invalid_argument("unknown option: " + key);
        }
        auto names = demo::expand_scenarios(scenario);
        if (output.empty())
            output = "outputs/cpp/" + scenario;
        std::vector<demo::Scene> scenes;
        for (const auto &name : names) {
            auto scene = demo::make_scene(name, iterations, seed);
            double length = 0;
            for (const auto &p : scene.paths)
                length += p.length;
            std::cout << std::fixed << std::setprecision(2) << name << ": " << length << " m, "
                      << scene.paths.size() << " legs, " << scene.planning_ms << " ms";
            if (scene.iterations)
                std::cout << ", " << scene.nodes << " nodes, " << scene.rewires << " rewires";
            std::cout << '\n';
            scenes.push_back(std::move(scene));
        }
        demo::save_visualization(scenes, output);
        std::cout
            << "Open in a browser: " << std::filesystem::absolute(output + "/index.html")
            << "\nCSV routes are beside the HTML. Animation is by distance, not flight time.\n";
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
