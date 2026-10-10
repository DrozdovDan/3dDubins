# 3D Dubins for aerial logistics

A small **C++17 planner with a Python interface** for **Owen, Beard & McLain (2014)** trajectories and **Lim et al. (2024)** terrain-aware planning between safe loiter circles. A readable Python reference remains available; plotting is optional.

![Two-leg route animation](visuals/two_leg_route.gif)

## Run

From this folder, build the fast backend once (Python 3.10+):

```bash
# Use an installed GCC, Clang, MSVC Developer shell or Zig compiler.
# If none is available, install a portable compiler:
python -m pip install ziglang
python -m dubins_airplane.build_native
```

Then run the existing examples:

```bash
python -m examples.example
python -m unittest discover -s tests -v
```

For plots and GIF generation:

```bash
python -m pip install -r requirements.txt
python -m examples.example --plot
python -m examples.example --save-visuals
```

The examples write CSV routes to `outputs/`. Saved PNGs and GIFs are in `visuals/`.

For interactive **Rerun** visualization (3D, top view, aircraft animation and altitude profile):

```bash
python -m pip install -r rerun_scripts/requirements.txt
python -m rerun_scripts.dubins --scenario buildings --backend cpp
python -m rerun_scripts.dubins --scenario all --backend cpp --save
python -m rerun outputs/rerun/ridge.rrd
```

The launcher requires C++ by default: the Python code exports and visualizes results, not the search loop. See [Rerun scenarios and controls](rerun_scripts/README.md#сценарии-dubins-с-c-бэкендом).

Terrain scenarios use **10 × 10 m cells**, preserving the 1180 × 800 m map extent
(1180 × 1000 m for `ridge`). Rerun colors terrain cells by elevation in both views,
with a shared numeric legend. A dimension line measures the actual start-to-finish
distance in plan; route length and the distance between loiter centers are shown separately.
The yellow aircraft marker has a live altitude/readout above the map.

Two harder scenarios at the same map scale: **slalom** (three alternating barrier gaps)
and **mountain_pass** (two peaks, two barriers and a climb to a plateau).
[Slalom GIF](visuals/slalom.gif) · [Mountain-pass GIF](visuals/mountain_pass.gif).
These are captures of the actual Rerun Viewer, with C++ planning, seed 7 and 600 iterations:

```bash
python -m rerun_scripts.dubins --scenario slalom --backend cpp --gif
python -m rerun_scripts.dubins --scenario mountain_pass --backend cpp --gif
```

After building, the examples and Python API automatically use C++. No compiler or extra runtime package is needed to run the built library. Without a library, they fall back to the standard-library Python reference. Check the active backend with `python -c "from dubins_airplane import backend_name; print(backend_name())"`.

Set `DUBINS_BACKEND=cpp` to require C++ or `DUBINS_BACKEND=python` to force the reference. In PowerShell: `$env:DUBINS_BACKEND='cpp'`; in bash: `export DUBINS_BACKEND=cpp`. Rebuild after changing native sources, using the target platform/architecture. For a **C++-only executable** and CMake instructions, see [native/README.md](native/README.md).

## Use it in Python

```python
from math import radians
from dubins_airplane import Pose, plan_dubins_airplane

start = Pose(0, 0, 120, radians(0))
goal = Pose(500, 250, 500, radians(90))
path = plan_dubins_airplane(start, goal, 60, radians(15))
print(path.altitude_case, path.word, path.radius, path.extra_turns, path.length)
points = path.sample(5)
```

`x` is east, `y` north, `z` upward (metres); yaw is in radians from east. `L`, `R`, and `S` mean left turn, right turn, and straight. `path.minimum_radius` is the input limit; `path.radius` is the actual radius.

The construction follows the chapter's three altitude cases:

- **Low:** choose the shortest of four CSC paths and fit a constant climb/descent angle.
- **Medium:** insert an additional arc and fit its displacement angle by bisection.
- **High:** add complete turns and fit a larger radius by bisection, at the maximum climb/descent angle.

The flight-path angle is constant within a leg. Extension is placed at the start for climb and at the end for descent. `minimum_turn_radius(speed, bank_angle)` also implements Equation (19); arguments are metres/second and radians.

## Explore

- [`example.py`](examples/example.py): climb to a high transfer point, then descend.
- [`example_arrival_heading.py`](examples/example_arrival_heading.py): same destination, different arrival headings.
- [`example_climb_limit.py`](examples/example_climb_limit.py): compare the low, medium, and high constructions.
- [`example_delivery_mission.py`](examples/example_delivery_mission.py): several preset delivery-area waypoints.
- [`example_obstacles.py`](examples/example_obstacles.py): four Dubins RRT* scenarios: a hill with a forbidden volume, several buildings, a mountain ridge, and a climb to higher terrain.
- [`benchmark_planner.py`](examples/benchmark_planner.py): repeatable first-solution and full-search timing for those scenarios.
- [Visual guide](docs/VISUAL_GUIDE.md), [method and paper guide](docs/KNOWLEDGE_GUIDE.md), [paper sources](papers/README.md), and [other implementations](docs/IMPLEMENTATIONS.md).

Source: [the chapter](https://scholarsarchive.byu.edu/facpub/1900/) and [NTNU reference code](https://github.com/ntnu-arl/DubinsAirplane). This implements the chapter's trajectory construction, not its guidance controller. Like the chapter, the base families are `RSR`, `RSL`, `LSR`, `LSL`; `RLR`/`LRL` are excluded. Nearby poses can require those families or have no solution on this construction; the reference examples use a conservative horizontal separation of at least `6 * minimum_radius`. Unsupported length equations raise `ValueError` rather than returning the old full-turn approximation.

## Plan around terrain and obstacles

```bash
python -m examples.example_obstacles
python -m examples.example_obstacles --plot
python -m examples.example_obstacles --save-visuals
python -m examples.example_obstacles --scenario buildings --plot
python -m examples.example_obstacles --scenario ridge --plot
python -m examples.example_obstacles --scenario climb --plot
python -m examples.example_obstacles --scenario all --save-visuals
```

To measure planner speed without plotting, CSV export, or GIF encoding:

```bash
python -m examples.benchmark_planner --iterations 600
# Compare both backends on identical samples, without reducing the budget.
python -m examples.benchmark_planner --backend both --iterations 600 --repeats 3
# Check the full-call <100 ms target without shortening the search.
python -m examples.benchmark_planner --backend cpp --iterations 600 --repeats 50 --max-ms 100
# Optional: repeat with several seeds for timing variability.
python -m examples.benchmark_planner --scenario hill_box --iterations 600 --repeats 5
```

The benchmark reports time to the first feasible path and total RRT* call time (median and range) for each scenario. It prepares the map before timing and uses consecutive random seeds. These are wall-clock measurements on the current computer, not flight-time estimates or an optimality guarantee.

After the additional native optimizations, the development machine (Windows 11, Python 3.12.13, Zig 0.16.0) completed **all 200 planning calls below 100 ms**: 600 iterations, 50 seeds per scenario, seeds 7–56. Full-call times, including Python validation, sampling and native data transfer, but excluding map construction, visualization and warm-up:

| Scenario | Median | Worst observed | Below 100 ms |
|---|---:|---:|---:|
| Hill + box | 30 ms | 35.57 ms | 50/50 |
| Buildings | 32 ms | 46.88 ms | 50/50 |
| Ridge | 54 ms | 67.18 ms | 50/50 |
| Climb | 32 ms | 44.48 ms | 50/50 |

The optimizations use lazy exact parent selection with conservative cost bounds, cached curve prefixes/trigonometry, shared root-bracket endpoint values and reusable sorting buffers. The iteration count, Owen families, bisection tolerances and continuous collision-check subdivision are unchanged. On 120 paired calls (600 iterations, seeds 7–36), the optimized backend returned **bit-identical** paths, cost histories and node/rewire counts compared with the previous C++ binary. Differential tests also cover 2,000 state pairs, terrain offsets and routes independently checked by Python. Across different compilers/math libraries or the Python reference, floating-point ties can still resolve differently.

`--max-ms 100` checks every full call against the strict `<100 ms` threshold and exits with status 1 if any call misses it. It is a benchmark assertion, **not** an early-stop setting. These measurements are not a hard real-time guarantee for arbitrary maps, larger budgets, other machines or operating-system load. `time_limit` remains a soft budget, not a guaranteed 100 ms deadline.

![Terrain and obstacle avoidance](visuals/obstacle_route.gif)

Read the Russian plots left to right: **map → 3D context → altitude profile**. The entire found route is blue; orange hatched areas are forbidden volumes with their height ranges labeled. Teal dashed circles are safe start/goal loiters. A direct Dubins connection between the **same endpoint states** is red dashed if blocked, grey dashed if free. A red cross locates a blocked sample. Below, blue altitude stays inside the green permitted corridor; grey-brown ground is the terrain beneath this route. Small numbers mark joins, not separate missions. GIFs synchronize the moving marker across all three panels.

The default `hill_box` scenario avoids a forbidden volume over a hill. `buildings` demonstrates lateral avoidance on flat terrain, `ridge` avoids high terrain without artificial obstacles, and `climb` reaches a higher destination with a valid direct path. Each example lets RRT* choose intermediate states; none are hand-picked. Use `--seed` and `--iterations` to change the reproducible search. The default CSV is `outputs/obstacle_route.csv`; the other stems are `obstacle_buildings`, `obstacle_ridge`, and `obstacle_climb`. PNGs and GIFs use the same stems in `visuals/`.

```python
from math import radians
from dubins_airplane import ElevationMap, FlightWorld, BoxObstacle, plan_safe_rrt_star

terrain = ElevationMap.from_function(-180, -400, 10, 118, 80, lambda x, y: 0)
world = FlightWorld(terrain, min_clearance=40, max_clearance=110,
                    obstacles=[BoxObstacle(320, -130, 0, 480, 130, 300)])
start = world.choose_loiter(0, 0, radius=40)
goal = world.choose_loiter(800, 0, radius=40)
route = plan_safe_rrt_star(world, start, goal, radius=40,
                           max_flight_path_angle=radians(15), seed=7)
points = route.sample(5)
```

Terrain is a grid of local metre heights (`heights[row][column]`), not latitude/longitude. Clearance is based on Euclidean terrain offsets, not just height above the cell below. The checker conservatively bounds whole curve intervals, so it cannot skip a thin box between point samples. Unknown map space is forbidden.

This independently implements Lim's planning idea, not the complete ROS/PX4 system. Its raster cells, finite circle samples and CSC steering are conservative approximations; a search failure is **not** a proof that no route exists. Exact differences are in the [method guide](docs/KNOWLEDGE_GUIDE.md#31-terrain-and-obstacle-planning-lim-et-al).

The standalone `plan_dubins_airplane` still ignores obstacles; use `plan_safe_rrt_star` for the supplied static map. Neither planner models wind, tracking error, aircraft size unless margins are supplied, landing, or vehicle-specific dynamics. This is not flight-certified software or a general globally shortest 3D solver. Local PDFs in `papers/` are Git-ignored until redistribution rights are checked.
